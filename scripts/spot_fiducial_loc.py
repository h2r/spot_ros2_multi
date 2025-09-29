#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf_transformations
import numpy as np


class SpotTf(Node):
    def __init__(self):
        super().__init__('spot_tf')

        self.tf_publisher = self.create_publisher(TransformStamped, 'spot_tf_compute', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # sleep for a while to wait for the tf listener to be ready
        time.sleep(1.0)

        self.timer = self.create_timer(0.1, self.publish_transform)

        self.spot_tf_not_exists = False

    def publish_transform(self):
        try:
            spot1_tf = self.tf_buffer.lookup_transform('spot/filtered_fiducial_111', 'spot/body', rclpy.time.Time())
            spot2_tf = self.tf_buffer.lookup_transform('spot2/filtered_fiducial_111', 'spot2/body', rclpy.time.Time())
        except Exception as e:
            if not self.spot_tf_not_exists:
                self.get_logger().info('Waiting for transforms...')
                self.spot_tf_not_exists = True
            return
        if self.spot_tf_not_exists:
            self.get_logger().info('Transforms found, publishing relative transform.')
            self.spot_tf_not_exists = False

        spot1_R = tf_transformations.quaternion_matrix([
            spot1_tf.transform.rotation.x,
            spot1_tf.transform.rotation.y,
            spot1_tf.transform.rotation.z,
            spot1_tf.transform.rotation.w
        ])
        spot2_R = tf_transformations.quaternion_matrix([
            spot2_tf.transform.rotation.x,
            spot2_tf.transform.rotation.y,
            spot2_tf.transform.rotation.z,
            spot2_tf.transform.rotation.w
        ])
        spot1_R[:3, 3] = [
            spot1_tf.transform.translation.x,
            spot1_tf.transform.translation.y,
            spot1_tf.transform.translation.z
        ]
        spot2_R[:3, 3] = [
            spot2_tf.transform.translation.x,
            spot2_tf.transform.translation.y,
            spot2_tf.transform.translation.z
        ]

        spot1_to_spot2_R = np.linalg.solve(spot1_R, spot2_R)

        spot1_to_spot2 = TransformStamped()
        spot1_to_spot2.header.stamp = spot1_tf.header.stamp
        spot1_to_spot2.header.frame_id = 'spot/body'
        spot1_to_spot2.child_frame_id = 'spot2/body'
        spot1_to_spot2.transform.translation.x = spot1_to_spot2_R[0, 3]
        spot1_to_spot2.transform.translation.y = spot1_to_spot2_R[1, 3]
        spot1_to_spot2.transform.translation.z = spot1_to_spot2_R[2, 3]

        relative_quat = tf_transformations.quaternion_from_matrix(spot1_to_spot2_R)
        spot1_to_spot2.transform.rotation.x = relative_quat[0]
        spot1_to_spot2.transform.rotation.y = relative_quat[1]
        spot1_to_spot2.transform.rotation.z = relative_quat[2]
        spot1_to_spot2.transform.rotation.w = relative_quat[3]

        self.tf_publisher.publish(spot1_to_spot2)


def main(args=None):
    rclpy.init(args=args)
    spot_tf = SpotTf()
    rclpy.spin(spot_tf)
    spot_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



