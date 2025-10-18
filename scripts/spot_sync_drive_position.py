#!/usr/bin/env python3

import copy
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs  # Required for PoseStamped transformations
import tf_transformations as tf_trans
import numpy as np
import threading

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Transform
from builtin_interfaces.msg import Duration as DurationMsg
from spot_msgs.action import Trajectory

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
    Multi-robot synchronized tracking controller using trajectory actions.

    This node:
    - Subscribes to /multi_spot/cmd_vel for formation velocity commands
    - Integrates velocities to generate trajectory waypoints
    - Sends synchronized trajectory goals to each robot via actions
    - Monitors feedback to maintain formation synchronization
    """

    def __init__(self, spot_names):
        super().__init__('spot_sync_tracking')

        self.spot_names = spot_names
        self.base_spot_name = spot_names[0]

        self.mock = False

        # Create action clients for each robot
        self.trajectory_clients = {}
        self.trajectory_goals = {}
        self.trajectory_futures = {}
        self.trajectory_feedback = {}

        self.pivot_to_robot_R: Optional[dict[str, np.ndarray]] = None

        for spot_name in spot_names:
            client = ActionClient(self, Trajectory, f"/{spot_name}/trajectory")
            self.trajectory_clients[spot_name] = client
            self.trajectory_feedback[spot_name] = {"status": "idle", "message": ""}
            # create a cmd_vel subscriber for each robot to reset pivot if their individual location is changed
            self.create_subscription(Twist, f"/{spot_name}/cmd_vel", self._single_cmd_vel_callback, 1)
            self.get_logger().info(f"Created action client for {spot_name}")

        # TF setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(nanoseconds=int(5e8)))
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
        self.get_logger().info("Single_cmd_vel_callback called - Reset multi-robot pivot point.")
        self._reset_pivot()

    def _cmd_vel_callback(self, vel: Twist):
        """Store the latest velocity command"""
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
        Cancel all active trajectory goals to stop the robots.
        This is called when cmd_vel timeout is detected.
        """
        for spot_name in self.spot_names:
            # Cancel any active goal for this robot
            if spot_name in self.trajectory_goals and self.trajectory_goals[spot_name] is not None:
                goal_handle = self.trajectory_goals[spot_name]
                goal_handle.cancel_goal_async()
                self.get_logger().info(f"Cancelling active trajectory for {spot_name}")
                # Clear the goal handle
                self.trajectory_goals[spot_name] = None
            else:
                self.get_logger().debug(f"No active trajectory to cancel for {spot_name}")

    def _compute_target_pose_for_robot(self, spot_name: str, robot_R: np.ndarray,
                                      robot_pivot_R: np.ndarray, robot_pivot: np.ndarray, cmd_vel: Twist):
        """
        Compute target pose for a single robot based on formation velocity command.

        Args:
            spot_name: Name of the robot
            robot_R: Current robot transformation matrix
            robot_pivot_R: Formation pivot transformation matrix
            robot_pivot: Formation pivot position
            cmd_vel: Formation velocity command

        Returns:
            PoseStamped in robot's body frame for the trajectory goal
        """
        assert self.pivot_to_robot_R is not None, "pivot_to_robot_R not computed"
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

        # Create PoseStamped in body frame
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = target_z
        target_pose.pose.orientation.x = target_quat[0]
        target_pose.pose.orientation.y = target_quat[1]
        target_pose.pose.orientation.z = target_quat[2]
        target_pose.pose.orientation.w = target_quat[3]

        tf = TransformStamped()
        tf.header = copy.deepcopy(target_pose.header)
        tf.header.frame_id = "map"
        tf.child_frame_id = f"{spot_name}/target_pose"
        tf.transform.translation.x = target_pose.pose.position.x
        tf.transform.translation.y = target_pose.pose.position.y
        tf.transform.translation.z = target_pose.pose.position.z
        tf.transform.rotation = target_pose.pose.orientation
        self.tf_publisher.sendTransform(tf)
        self.get_logger().info(f"Target for {spot_name}: ({target_x:.2f}, {target_y:.2f}, {np.degrees(target_yaw):.1f}deg)")

        # transform target pose to robot frame
        target_pose_body = self.tf_buffer.transform(target_pose, f"{spot_name}/body")
        target_pose_body.header.frame_id = "body" # address spot ros2 msg no namespace issue

        self.get_logger().debug(
            f"{spot_name}: target=({target_x:.2f}, {target_y:.2f}, {np.degrees(target_yaw):.1f}deg). "
        )

        return target_pose_body

    def _reset_pivot(self):
        """Reset the pivot transforms to force recomputation on next cmd_vel"""
        with self.cmd_vel_lock:
            self.last_cmd_vel_time = None
            self.pivot_to_robot_R = None
            self.get_logger().info("Pivot transforms reset")

    def _update_trajectory_callback(self):
        """
        Periodic callback to generate and send trajectory goals to all robots.
        This ensures continuous motion tracking of the cmd_vel input.
        """
        if self.cmd_vel_lock.locked():
            return # Avoid overlapping calls
        with self.cmd_vel_lock:
            cmd_vel = self.current_cmd_vel
            last_time = self.last_cmd_vel_time

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

        # Generate and send trajectory goals for each robot
        for spot_name in self.spot_names:
            if self.pivot_to_robot_R is None:
                raise ValueError("pivot_to_robot_R not computed yet")

            # adjust the robot position to be consistent with the inital pivot offset when the
            # cmd_vel was first received
            adjusted_robot_R = robot_pivot_R @ self.pivot_to_robot_R[spot_name]
            # Compute target pose
            target_pose = self._compute_target_pose_for_robot(
                spot_name, adjusted_robot_R,
                robot_pivot_R, robot_pivot, cmd_vel
            )

            # Create trajectory goal
            goal = Trajectory.Goal()
            goal.target_pose = target_pose
            self.get_logger().info(f"   Goal for {spot_name}: {goal.target_pose.pose}")
            # Convert duration to seconds and nanoseconds
            duration_sec = int(self.trajectory_duration)
            duration_nsec = int((self.trajectory_duration - duration_sec) * 1e9)
            goal.duration = DurationMsg(sec=duration_sec, nanosec=duration_nsec)
            goal.precise_positioning = True
            goal.disable_obstacle_avoidance = False

            # Send goal asynchronously
            if self.mock: continue
            client = self.trajectory_clients[spot_name]
            if not client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn(f"Action server for {spot_name} not available")
                continue

            # Send goal with feedback callback
            send_goal_future = client.send_goal_async(
                goal,
                feedback_callback=lambda fb, name=spot_name: self._trajectory_feedback_callback(name, fb)
            )
            send_goal_future.add_done_callback(
                lambda future, name=spot_name: self._trajectory_response_callback(name, future)
            )

            self.get_logger().debug(f"Sent trajectory goal to {spot_name}")

    def _trajectory_feedback_callback(self, robot_name, feedback_msg):
        """Process feedback from trajectory action"""
        feedback = feedback_msg.feedback
        self.trajectory_feedback[robot_name] = {
            "status": "executing",
            "message": feedback.feedback
        }
        self.get_logger().debug(f"{robot_name} feedback: {feedback.feedback}")

    def _trajectory_response_callback(self, robot_name, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"{robot_name} trajectory goal rejected!")
            self.trajectory_feedback[robot_name] = {
                "status": "rejected",
                "message": "Goal rejected"
            }
            self.trajectory_goals[robot_name] = None
            return

        self.get_logger().debug(f"{robot_name} trajectory goal accepted")

        # Store the goal handle so we can cancel it later if needed
        self.trajectory_goals[robot_name] = goal_handle

        # Get result when trajectory completes
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, name=robot_name: self._trajectory_result_callback(name, future)
        )

    def _trajectory_result_callback(self, robot_name, future):
        """Handle trajectory completion result"""
        result = future.result().result

        # Clear the goal handle since execution is complete
        self.trajectory_goals[robot_name] = None

        if result.success:
            self.get_logger().debug(f"{robot_name} trajectory completed: {result.message}")
            self.trajectory_feedback[robot_name] = {
                "status": "completed",
                "message": result.message
            }
        elif result.message == "timeout":
            self.get_logger().info(f"{robot_name} trajectory finished and canceled with new goal (expected): {result.message}")
            self.trajectory_feedback[robot_name] = {
                "status": "timeout",
                "message": result.message
            }
        else:
            self.get_logger().warn(f"{robot_name} trajectory failed: {result.message}")
            self.trajectory_feedback[robot_name] = {
                "status": "failed",
                "message": result.message
            }

    def get_formation_status(self):
        """Get synchronization status of all robots"""
        statuses = [fb["status"] for fb in self.trajectory_feedback.values()]
        return {
            "all_executing": all(s == "executing" for s in statuses),
            "any_failed": any(s == "failed" for s in statuses),
            "details": self.trajectory_feedback
        }


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
