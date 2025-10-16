#!/usr/bin/env python3
from collections import defaultdict
import time
from typing import List
import rclpy
from rclpy.node import Node, Client

from spot_msgs.srv import GraphNavSetLocalization, GraphNavUploadGraph

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf_transformations
import numpy as np
import os


class SpotGraphNavLocalization(Node):
    def __init__(self, spots=["spot", "spot2"], pivot_spot="spot", graph_path="~/spot_configs/map/demo"):
        super().__init__('spot_graphnav_localization')

        self.upload_graph_clients: List[Client] = []
        self.set_localization_clients: List[Client] = []
        self.spots = spots
        self.pivot_spot = pivot_spot
        self.graph_path = graph_path
        self.graph_path = os.path.expanduser(self.graph_path)

        # call upload graph nav service
        for spot in spots:
            self.upload_graph_clients.append(
                self.create_client(GraphNavUploadGraph, f'/{spot}/graph_nav_upload_graph')
            )
            while not self.upload_graph_clients[-1].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'/{spot}/graph_nav_upload_graph service not available, waiting again...')
            self.get_logger().info(f'/{spot}/graph_nav_upload_graph service available.')
            self.set_localization_clients.append(
                self.create_client(GraphNavSetLocalization, f'/{spot}/graph_nav_set_localization')
            )
            while not self.set_localization_clients[-1].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'/{spot}/graph_nav_set_localization service not available, waiting again...')
            self.get_logger().info(f'/{spot}/graph_nav_set_localization service available.')

        # publish graph nav map
        for i, spot in enumerate(spots):
            try:
                print(f'Uploading graph nav map and setting localization for {spot}...')
                upload_future = self.upload_graph_clients[i].call_async(GraphNavUploadGraph.Request(upload_filepath=self.graph_path))
                rclpy.spin_until_future_complete(self, upload_future, timeout_sec=10.0)
                if upload_future.result() is not None:
                    print("Graph nav map uploaded.")
                else:
                    self.get_logger().error(f'Failed to upload graph for {spot}')
                    continue

                loc_future = self.set_localization_clients[i].call_async(GraphNavSetLocalization.Request(method="fiducial", waypoint_id=""))
                rclpy.spin_until_future_complete(self, loc_future, timeout_sec=10.0)
                if loc_future.result() is not None:
                    self.get_logger().info(f'Localization set successfully for {spot}')
                else:
                    self.get_logger().error(f'Failed to set localization for {spot}')
                print(f'Graph nav map uploaded and localization set for {spot}.')
            except Exception as e:
                print(f'Error occurred while uploading graph nav map or setting localization for {spot}: {e}')

        self.tf_publisher = self.create_publisher(TransformStamped, 'spot_tf_compute', 10)
        self.robot_tf_publishers = {}
        for spot in spots:
            self.robot_tf_publishers[spot] = self.create_publisher(TransformStamped, f'/{spot}/localization_pose_adjusted', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.icp_alignment_offset = defaultdict(lambda: np.eye(4))
        self.icp_alignment_listener = self.create_subscription(TransformStamped, "/multi_spot/pc2_realign_tf", self.realignment_cb, 10)
        # sleep for a while to wait for the tf listener to be ready
        time.sleep(1.0)

        self.timer = self.create_timer(0.1, self.publish_transform)
    
    def realignment_cb(self, msg: TransformStamped):
        if msg.header.frame_id.rstrip("_aligned") != self.pivot_spot:
            print("not pivot. pivot should be:", self.pivot_spot)
            return
        spot2 = msg.child_frame_id.rstrip("_aligned")
        if spot2 not in self.spots:
            print(spot2, "not found in the list.")
            return

        self.get_logger().debug(f"Updating spot {spot2} GraphNav offset with lidar point cloud icp")
        offset_mat = self.tf_to_matrix(msg)
        offset_mat[2, 3] = 0
        self.icp_alignment_offset[spot2] = offset_mat
    
    def tf_to_matrix(self, tf: TransformStamped) -> np.ndarray:
        quat = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
        trans = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
        mat = tf_transformations.quaternion_matrix(quat)
        mat[:3, 3] = trans
        return mat

    def matrix_to_tf(self, mat: np.ndarray, parent_frame: str, child_frame: str) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = mat[0, 3]
        t.transform.translation.y = mat[1, 3]
        t.transform.translation.z = mat[2, 3]
        quat = tf_transformations.quaternion_from_matrix(mat)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t
    
    def publish_transform(self):
        for spot in self.spots:
            try:
                spot_pivot_tf = self.tf_buffer.lookup_transform('map', f'{self.pivot_spot}/body', rclpy.time.Time())
                spot_curr_tf = self.tf_buffer.lookup_transform('map', f'{spot}/body', rclpy.time.Time())
                spot_pivot_R = self.tf_to_matrix(spot_pivot_tf)
                spot_curr_R = self.tf_to_matrix(spot_curr_tf)
            except Exception as e:
                self.get_logger().info(f'Waiting for transforms for {spot}...')
                return
            
            # publish map to body transform with ICP correction and without ICP correction
            map_T_spot_icp = spot_curr_R @ self.icp_alignment_offset[self.pivot_spot]
            map_T_spot_msg = self.matrix_to_tf(map_T_spot_icp, 'map', f'{spot}/body')
            # map_T_spot_orig_msg = self.matrix_to_tf(spot_curr_R, 'map', f'{spot}/body_orig')
            self.tf_publisher.publish(map_T_spot_msg)
            # self.tf_publisher.publish(map_T_spot_orig_msg)
            self.robot_tf_publishers[spot].publish(map_T_spot_msg)

            # publish spot1 to spot2 transform with ICP correction and without ICP correction
            # if spot != self.pivot_spot:
                # spot1_T_spot2_icp = np.linalg.inv(spot_pivot_R) @ self.icp_alignment_offset[spot] @ spot_curr_R
                # spot1_T_spot2 = np.linalg.solve(spot_pivot_R, spot_curr_R)  # without ICP correction
                # tf_msg1 = self.matrix_to_tf(spot1_T_spot2_icp, f'{self.pivot_spot}/body', f'{spot}/body')
                # tf_msg2 = self.matrix_to_tf(spot1_T_spot2, f'{self.pivot_spot}/body', f'{spot}/body_orig')
                # self.tf_publisher.publish(tf_msg1)
                # self.tf_publisher.publish(tf_msg2)


def main(args=None):
    rclpy.init(args=args)
    spot_tf = SpotGraphNavLocalization()
    rclpy.spin(spot_tf)
    spot_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



