#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs  # Required for PoseStamped transformations
import tf_transformations as tf_trans
import numpy as np
import threading

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Transform
from std_srvs.srv import Trigger

def tf_to_matrix(tf: TransformStamped):
    """Convert TransformStamped to 4x4 transformation matrix"""
    t = tf.transform.translation
    q = tf.transform.rotation
    quat = [q.x, q.y, q.z, q.w]
    R = np.eye(4)
    R[:3, :3] = tf_trans.quaternion_matrix(quat)[:3, :3]
    R[:3, 3] = [t.x, t.y, t.z]
    return R

def matrix_to_tf(R: np.ndarray, parent_frame: str, child_frame: str, node: Node) -> TransformStamped:
    """Convert 4x4 transformation matrix to TransformStamped"""
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


class SpotSyncTracking(Node):
    """
    Multi-robot synchronized tracking controller using fire-and-forget navigate_to_pose.

    This node:
    - Subscribes to /multi_spot/cmd_vel for formation velocity commands
    - Integrates velocities to generate trajectory waypoints
    - Sends synchronized pose commands to each robot via navigate_to_pose topic
    - Uses stop service for immediate halt
    """

    def __init__(self, spot_names):
        super().__init__('spot_sync_tracking')

        self.spot_names = spot_names
        self.base_spot_name = spot_names[0]

        self.mock = False

        # Create publishers and service clients for each robot
        self.navigate_publishers = {}
        self.stop_clients = {}

        self.pivot_to_robot_R: Optional[dict[str, np.ndarray]] = None

        for spot_name in spot_names:
            # Create publisher for navigate_to_pose
            pub = self.create_publisher(PoseStamped, f"/{spot_name}/navigate_to_pose", 1)
            self.navigate_publishers[spot_name] = pub

            # Create service client for stop
            stop_client = self.create_client(Trigger, f"/{spot_name}/stop")
            self.stop_clients[spot_name] = stop_client

            # create a cmd_vel subscriber for each robot to reset pivot if their individual location is changed
            self.create_subscription(Twist, f"/{spot_name}/cmd_vel", self._single_cmd_vel_callback, 1)
            self.get_logger().info(f"Created navigate_to_pose publisher for {spot_name}")

        # TF setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_publisher = tf2_ros.TransformBroadcaster(self)

        # Velocity command subscriber
        self.vel_subscriber = self.create_subscription(
            Twist,
            "/multi_spot/cmd_vel",
            self._cmd_vel_callback,
            1
        )

        # Parameters for trajectory generation
        self.trajectory_duration = 1.0  # seconds - how far ahead to plan
        self.max_linear_speed = 0.3 # m/s
        self.max_angular_speed = 0.20  # rad/s
        self.stop_traj_timeout = 0.5  # seconds - stop if no cmd_vel received
        self.reset_pivot_timeout = 5  # seconds - reset pivot if no cmd_vel received

        # Current velocity command
        self.current_cmd_vel = Twist()
        self.last_cmd_vel_time = None
        self.cmd_vel_lock = threading.Lock()
        self.pivot_lock = threading.Lock()
        self.stop_command_sent = False  # Track if we've already sent stop

        # Timer for periodic trajectory updates
        # Update rate should be less than 1/trajectory_duration to avoid flooding
        self.update_rate = 10.0  # Hz - how often to send new trajectories
        self.update_timer = self.create_timer(1.0 / self.update_rate, self._update_trajectory_callback)

        self.get_logger().info(f"Initialized SpotSyncTracking for robots: {spot_names}")
    
    def _single_cmd_vel_callback(self, vel: Twist):
        """Callback for single robot cmd_vel - not used in multi-robot setup"""
        self._reset_pivot()

    def _cmd_vel_callback(self, vel: Twist):
        """Store the latest velocity command"""
        if self.cmd_vel_lock.locked():
            return # Avoid overlapping calls
        with self.cmd_vel_lock:
            self.current_cmd_vel = vel
            self.last_cmd_vel_time = self.get_clock().now()
            self.stop_command_sent = False  # Reset flag when new cmd_vel arrives
            self.get_logger().info(
                f"Received cmd_vel: linear=({vel.linear.x:.2f}, {vel.linear.y:.2f}), "
                f"angular={vel.angular.z:.2f}"
            )

    def _get_robot_transforms(self):
        """Get current transforms for all robots in the map frame, dict of R and 3D locations"""
        robot_tfs = {}
        robot_locs = {}

        try:
            for spot_name in self.spot_names:
                spot_pose = self.tf_buffer.lookup_transform(
                    "map",
                    f"{spot_name}/body",
                    rclpy.time.Time()
                )
                robot_tfs[spot_name] = tf_to_matrix(spot_pose)
                robot_locs[spot_name] = robot_tfs[spot_name][:3, 3]
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None, None

        return robot_tfs, robot_locs

    def _compute_tf_to_pivot(self, robot_tfs, robot_locs):
        """Compute and store transforms from each robot to the pivot frame"""
        if self.pivot_to_robot_R is not None: return
        print(robot_locs)
        with self.pivot_lock:
            self.get_logger().info("--- New Command Session -- computing pivot transforms")
            self.pivot_to_robot_R = {}
            robot_pivot = np.array(list(robot_locs.values())).mean(axis=0)
            robot_pivot_R = np.eye(4)
            robot_pivot_R[:3, :3] = robot_tfs[self.base_spot_name][:3, :3]
            robot_pivot_R[:3, 3] = robot_pivot
            print(f"Robot pivot at: {robot_pivot}")
            for spot_name in self.spot_names:
                pivot_to_robot_R = np.linalg.solve(robot_pivot_R, robot_tfs[spot_name])
                print(f"{spot_name} pivot offset: {pivot_to_robot_R[:3,3]}")
                print(f"{spot_name} transform:\n{robot_tfs[spot_name]}")
                self.pivot_to_robot_R[spot_name] = pivot_to_robot_R

    def _compute_formation_pivot(self, robot_tfs, robot_locs):
        """
        Compute the pivot point for the robot formation.
        The pivot is at the centroid of all robots, with orientation from base robot.
        """
        robot_pivot = np.array(list(robot_locs.values())).mean(axis=0)
        robot_pivot_R = np.eye(4)
        robot_pivot_R[:3, :3] = robot_tfs[self.base_spot_name][:3, :3]
        robot_pivot_R[:3, 3] = robot_pivot

        # Broadcast the pivot frame for visualization
        robot_pivot_tf = matrix_to_tf(robot_pivot_R, "map", "robot_pivot", self)
        self.tf_publisher.sendTransform(robot_pivot_tf)

        return robot_pivot_R, robot_pivot

    def _send_stop_command(self):
        """
        Call stop service for all robots to halt them immediately.
        This is called when cmd_vel timeout is detected.
        """
        for spot_name in self.spot_names:
            stop_client = self.stop_clients[spot_name]
            if stop_client.service_is_ready():
                request = Trigger.Request()
                future = stop_client.call_async(request)
                self.get_logger().info(f"Sent stop command to {spot_name}")
            else:
                self.get_logger().warn(f"Stop service not ready for {spot_name}")

    def _compute_target_pose_for_robot(self, spot_name: str, robot_R: np.ndarray,
                                      robot_pivot_R: np.ndarray, robot_pivot: np.ndarray, cmd_vel: Twist,
                                      req_stamp: rclpy.time.Time) -> PoseStamped:
        """
        Compute target pose for a single robot based on formation velocity command.

        Args:
            spot_name: Name of the robot
            robot_R: Current robot transformation matrix
            robot_pivot_R: Formation pivot transformation matrix
            robot_pivot: Formation pivot position
            cmd_vel: Formation velocity command

        Returns:
            PoseStamped in map frame with timestamp for synchronized arrival
        """
        assert self.pivot_to_robot_R is not None, "pivot_to_robot_R not computed"
        assert req_stamp is not None and type(req_stamp) is rclpy.time.Time, "req_stamp must be provided"
        # Get velocity components in the pivot's body frame
        body_linear_vel = np.array([cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z])
        angular_speed = cmd_vel.angular.z

        # Transform velocity from pivot body frame to map frame
        # The rotation matrix from the pivot gives us the orientation
        body_to_map_rotation = robot_pivot_R[:3, :3]
        map_linear_vel = body_to_map_rotation @ body_linear_vel

        # compute new location of the pivot after trajectory_duration
        dt = self.trajectory_duration
        new_pivot = robot_pivot + map_linear_vel * dt
        rot_mat = np.eye(4)
        # rotate the pivot around the z axis
        if abs(angular_speed) > 0.001:
            angle = angular_speed * dt
            rot_mat[:3, :3] = [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle),  np.cos(angle), 0],
                [0,             0,              1]
            ]
        new_pivot_R = robot_pivot_R @ rot_mat
        new_pivot_R[:3, 3] = new_pivot

        target_R = new_pivot_R @ self.pivot_to_robot_R[spot_name]
        target_x, target_y, target_z = target_R[:3, 3]
        target_quat = tf_trans.quaternion_from_matrix(target_R)
        target_yaw = tf_trans.euler_from_quaternion(target_quat)[2]

        # Create PoseStamped in map frame first
        target_pose_map = PoseStamped()
        target_pose_map.header.frame_id = "map"
        target_pose_map.header.stamp = rclpy.time.Time().to_msg()
        target_pose_map.pose.position.x = target_x
        target_pose_map.pose.position.y = target_y
        target_pose_map.pose.position.z = target_z
        target_pose_map.pose.orientation.x = target_quat[0]
        target_pose_map.pose.orientation.y = target_quat[1]
        target_pose_map.pose.orientation.z = target_quat[2]
        target_pose_map.pose.orientation.w = target_quat[3]

        # Transform to robot's body frame
        try:
            target_pose = self.tf_buffer.transform(target_pose_map, f"{spot_name}/body", timeout=Duration(nanoseconds=int(2e8)))
            # Update timestamp to when robot should arrive (for synchronized execution)
            target_pose.header.stamp = (req_stamp + Duration(seconds=dt)).to_msg()
            # Keep frame_id as body for the command
            target_pose.header.frame_id = "body"  # Remove namespace for spot_ros2 compatibility

            self.get_logger().debug(f"Transformed target pose to {spot_name}/body frame")
        except Exception as e:
            self.get_logger().error(f"Failed to transform target pose to {spot_name}/body: {e}. Using map frame.")
            target_pose = target_pose_map
            # Set timestamp for synchronized arrival
            target_pose.header.stamp = (self.get_clock().now() + Duration(seconds=dt)).to_msg()

        # Publish TF for visualization (in map frame)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = f"{spot_name}/target_pose"
        tf.transform.translation.x = target_x
        tf.transform.translation.y = target_y
        tf.transform.translation.z = target_z
        tf.transform.rotation.x = target_quat[0]
        tf.transform.rotation.y = target_quat[1]
        tf.transform.rotation.z = target_quat[2]
        tf.transform.rotation.w = target_quat[3]
        self.tf_publisher.sendTransform(tf)
        self.get_logger().info(f"Target for {spot_name}: ({target_x:.2f}, {target_y:.2f}, {np.degrees(target_yaw):.1f}deg) in map frame")

        self.get_logger().debug(
            f"{spot_name}: target=({target_x:.2f}, {target_y:.2f}, {np.degrees(target_yaw):.1f}deg). "
        )

        return target_pose

    def _reset_pivot(self):
        """Reset the pivot transforms to force recomputation on next cmd_vel"""
        with self.cmd_vel_lock:
            if self.last_cmd_vel_time is not None:
                self.last_cmd_vel_time = None
                self.pivot_to_robot_R = None
                self.stop_command_sent = True
                self.get_logger().info("Pivot transforms reset")

    def _update_trajectory_callback(self):
        """
        Periodic callback to generate and send trajectory goals to all robots.
        This ensures continuous motion tracking of the cmd_vel input.
        """
        with self.cmd_vel_lock:
            cmd_vel = self.current_cmd_vel
            last_time = self.last_cmd_vel_time
        req_stamp = self.get_clock().now()

        # Check for timeout - stop robots if no cmd_vel received recently
        if last_time is not None:
            time_since_last_cmd = (self.get_clock().now() - last_time).nanoseconds / 1e9
            if time_since_last_cmd > self.stop_traj_timeout:
                # Only send stop command once
                if not self.stop_command_sent:
                    self.get_logger().warn(
                        f"No cmd_vel received for {time_since_last_cmd:.2f}s (timeout: {self.stop_traj_timeout}s) - stopping robots"
                    )
                    self._send_stop_command()
                    self.stop_command_sent = True
                if time_since_last_cmd > self.reset_pivot_timeout:
                    self.get_logger().info(
                        f"No cmd_vel received for {time_since_last_cmd:.2f}s (reset timeout: {self.reset_pivot_timeout}s) - resetting pivot"
                    )
                    self._reset_pivot()
                return

        # If no cmd_vel ever received, don't send trajectories
        if last_time is None:
            return

        # If zero velocity, don't send new trajectories
        if (abs(cmd_vel.linear.x) < 0.01 and
            abs(cmd_vel.linear.y) < 0.01 and
            abs(cmd_vel.angular.z) < 0.01):
            return

        # Get current robot states
        robot_tfs, robot_locs = self._get_robot_transforms()
        if robot_tfs is None:
            return

        # Compute formation pivot
        robot_pivot_R, robot_pivot = self._compute_formation_pivot(robot_tfs, robot_locs)
        if self.pivot_to_robot_R is None:
            self._compute_tf_to_pivot(robot_tfs, robot_locs)

        # Generate and send trajectory poses for each robot (fire-and-forget)
        for spot_name in self.spot_names:
            if self.pivot_to_robot_R is None:
                raise ValueError("pivot_to_robot_R not computed yet")

            # adjust the robot position to be consistent with the inital pivot offset when the
            # cmd_vel was first received
            adjusted_robot_R = robot_pivot_R @ self.pivot_to_robot_R[spot_name]
            # Compute target pose
            target_pose = self._compute_target_pose_for_robot(
                spot_name, adjusted_robot_R,
                robot_pivot_R, robot_pivot, cmd_vel, req_stamp
            )

            # Publish pose to navigate_to_pose topic (fire-and-forget)
            if not self.mock:
                publisher = self.navigate_publishers[spot_name]
                publisher.publish(target_pose)
                self.get_logger().debug(f"Published navigate_to_pose for {spot_name}")


def main(args=None):
    rclpy.init(args=args)
    print("Initializing SpotSyncTracking...")

    # TODO: Make this configurable via launch parameters
    spot_names = ["spot", "spot2"]

    sync_tracking = SpotSyncTracking(spot_names)

    try:
        rclpy.spin(sync_tracking)
    except KeyboardInterrupt:
        pass
    finally:
        sync_tracking.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
