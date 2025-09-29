#!/usr/bin/env python3
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # sleep for a while to wait for the tf listener to be ready
        time.sleep(1.0)

        self.timer = self.create_timer(0.1, self.publish_transform)
    
    def publish_transform(self):
        for spot in self.spots:
            if spot == self.pivot_spot:
                continue
            try:
                spot_tf = self.tf_buffer.lookup_transform(f'{self.pivot_spot}/body', f'{spot}/body', rclpy.time.Time())
            except Exception as e:
                self.get_logger().info(f'Waiting for transforms for {spot}...')
                return

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f'{self.pivot_spot}/body'
            t.child_frame_id = f'{spot}/body'
            t.transform.translation.x = spot_tf.transform.translation.x
            t.transform.translation.y = spot_tf.transform.translation.y
            t.transform.translation.z = spot_tf.transform.translation.z
            t.transform.rotation = spot_tf.transform.rotation
            self.tf_publisher.publish(t)


def main(args=None):
    rclpy.init(args=args)
    spot_tf = SpotGraphNavLocalization()
    rclpy.spin(spot_tf)
    spot_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



