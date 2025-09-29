#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import tf_transformations as tf_trans
import numpy as np

from geometry_msgs.msg import Twist, TransformStamped

def tf_to_matrix(tf: TransformStamped):
    t = tf.transform.translation
    q = tf.transform.rotation
    quat = [q.x, q.y, q.z, q.w]
    R = np.eye(4)
    R[:3, :3] = tf_trans.quaternion_matrix(quat)[:3, :3]
    R[:3, 3] = [t.x, t.y, t.z]
    return R

def matrix_to_tf(R: np.ndarray, parent_frame: str, child_frame: str, node: Node) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    translation = R[:3, 3]
    rotation = tf_trans.quaternion_from_matrix(R)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return t


class SpotSyncDrive(Node):
    def __init__(self, spot_names):
        super().__init__('spot_sync_drive')
        self.spot_vel_topics = {
            spot_name: self.create_publisher(Twist, f"/{spot_name}/cmd_vel", 1)
            for spot_name in spot_names
        }
        self.base_spot_name = spot_names[0]
        # Reduce cache time - you're only using "now" transforms
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(nanoseconds=int(5e8)))  # 0.5 seconds
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_publisher = tf2_ros.TransformBroadcaster(self)

        self.vel_subscriber = self.create_subscription(Twist, "/multi_spot/cmd_vel", self.cmd_vel, 1)
        self.get_logger().info("initialized.")

    

    def get_relative_pose(self, robot1, robot2):
        tf_robots = self.tf_buffer.lookup_transform(robot2 + "/body", robot1 + "/body", time=rclpy.time.Time())
        q = tf_robots.transform.rotation
        quat = [q.x, q.y, q.z, q.w]

        # Convert to rotation matrix
        R = tf_trans.quaternion_matrix(quat)

        t = tf_trans.translation_matrix([tf_robots.transform.translation.x,
                                          tf_robots.transform.translation.y,
                                          tf_robots.transform.translation.z])

        R_inv = np.linalg.inv(R)
        t_inv = -t

        return (R, t, R_inv, t_inv)


    # FIXME: please clean up
    def cmd_vel(self, vel: Twist):
        # transform wrt to the r
        time = self.get_clock().now()
        global_linear_vel = np.array([vel.linear.x, vel.linear.y, vel.linear.z])
        angular_speed = np.array([vel.angular.x, vel.angular.y, vel.angular.z])
        self.get_logger().info("Received cmd_vel: linear={}, angular={}".format(global_linear_vel, angular_speed))
        assert(len(self.spot_vel_topics) == 2) # only support 2 spots

        # gather all robot transforms
        robot_tfs = {}
        robot_locs = {}
        for spot_name in self.spot_vel_topics.keys():
            spot_pose = self.tf_buffer.lookup_transform("map", f"{spot_name}/body", rclpy.time.Time())
            robot_tfs[spot_name] = tf_to_matrix(spot_pose)
            robot_locs[spot_name] = robot_tfs[spot_name][:3, 3]

        # define the robot pivot as the midpoint between all robots
        # for now, the direction is relative to the base robot
        robot_pivot = np.array(list(robot_locs.values())).mean(axis=0)
        robot_pivot_R = np.eye(4)
        robot_pivot_R[:3, :3] = robot_tfs[self.base_spot_name][:3, :3]
        robot_pivot_R[:3, 3] = robot_pivot

        robot_pivot_tf = matrix_to_tf(robot_pivot_R, "map", "robot_pivot", self)
        self.tf_publisher.sendTransform(robot_pivot_tf)

        max_linear_speed = 0.3
        max_angular_speed = 0.20
        # Cap the angular speed to avoid instability
        if np.abs(angular_speed[2]) > max_angular_speed:
            angular_speed[2] = max_angular_speed * np.sign(angular_speed[2])
            
        # compute each spot v and send it out to each robot.
        for spot_name, spot_publisher in self.spot_vel_topics.items():
            is_base_spot = (spot_name == self.base_spot_name)

            relative_R = np.linalg.inv(robot_tfs[spot_name]) @ robot_pivot_R
            relative_xyz = relative_R[:3, 3]
            # print("relative_R for {}:\n{}".format(spot_name, relative_R))
            # print("relative_R extracted xyz for {}:\n{}".format(spot_name, relative_xyz))
            # print("diff in xyz for {}: {}".format(spot_name, robot_locs[spot_name] - robot_pivot))
            radius = np.linalg.norm(robot_locs[spot_name] - robot_pivot)  # Distance between robots and midpoint
            new_linear_vel = relative_R[:3, :3] @ global_linear_vel

            linear_rotation_vel_magnitude = radius * angular_speed[2]
            # the linear velocity vector that is perpendicular to the radius vector in the horizontal plane
            linear_rotation_vel_vec = np.array([relative_xyz[1], -relative_xyz[0], 0.0])
            if np.linalg.norm(linear_rotation_vel_vec) > 1e-6:
                linear_rotation_vel_vec /= np.linalg.norm(linear_rotation_vel_vec)
            else:
                linear_rotation_vel_vec = np.array([0.0, 0.0, 0.0])

            linear_rotation_vel_vec *= linear_rotation_vel_magnitude
            
            # Bump up the x component a bit, since the robots don't respond equally to x and y velocities
            linear_rotation_vel_vec[0] *= (1.0 + np.abs(linear_rotation_vel_vec[1] * 0.05))

            # Cap the linear speed to avoid instability
            new_linear_vel = new_linear_vel + linear_rotation_vel_vec
            if (np.linalg.norm(new_linear_vel) > max_linear_speed):
                new_linear_vel = new_linear_vel / np.linalg.norm(new_linear_vel) * max_linear_speed            

            self.get_logger().info("For {}, radius: {}, linear_rotation_vel_vec: {}".format(
                spot_name, radius, linear_rotation_vel_vec)
            )

            new_twist_cmd = Twist()
            new_twist_cmd.linear.x = new_linear_vel[0]
            new_twist_cmd.linear.y = new_linear_vel[1]
            new_twist_cmd.linear.z = new_linear_vel[2]
            new_twist_cmd.angular.x = angular_speed[0]
            new_twist_cmd.angular.y = angular_speed[1]
            new_twist_cmd.angular.z = angular_speed[2]
            self.get_logger().info("{} linear speed: {}, angular speed: {}".format(
                spot_name, new_twist_cmd.linear, new_twist_cmd.angular)
            )
            spot_publisher.publish(new_twist_cmd)
        self.get_logger().info("")

def main(args=None):
    rclpy.init(args=args)
    print("initializing...")
    sync_drive = SpotSyncDrive(["spot", "spot2"])
    rclpy.spin(sync_drive)
    sync_drive.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
