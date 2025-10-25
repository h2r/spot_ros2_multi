#!/usr/bin/env python3

"""
Translates /spot/gripper_angle_cmd topic to /spot/set_gripper_angle service.

This allows non-blocking gripper angle commands via topic interface instead of
blocking service calls.
"""

import rclpy
from std_msgs.msg import Float32
from spot_msgs.srv import SetGripperAngle

from spot_service_translator import SpotServiceTranslator


def build_gripper_angle_request(msg: Float32) -> SetGripperAngle.Request:
    """
    Convert Float32 message to SetGripperAngle service request.

    Args:
        msg: Float32 message containing gripper angle in range [0, 90]

    Returns:
        SetGripperAngle.Request with gripper_angle set
    """
    request = SetGripperAngle.Request()
    request.gripper_angle = msg.data
    return request


def main(args=None):
    """Main function to launch the gripper angle translator."""
    rclpy.init(args=args)

    # Create the node to get parameters
    node = rclpy.create_node('gripper_angle_translator_param_reader')

    # Declare and get parameters
    node.declare_parameter('spot_name', 'spot')
    node.declare_parameter('update_rate', 0.1)

    spot_name = node.get_parameter('spot_name').get_parameter_value().string_value
    update_rate = node.get_parameter('update_rate').get_parameter_value().double_value

    node.destroy_node()

    # Create the service translator
    translator = SpotServiceTranslator(
        service_name=f"/{spot_name}/set_gripper_angle",
        topic_name=f"/{spot_name}/gripper_angle_cmd",
        service_type=SetGripperAngle,
        msg_type=Float32,
        request_builder=build_gripper_angle_request,
        update_rate=update_rate
    )

    try:
        rclpy.spin(translator)
    except KeyboardInterrupt:
        translator.get_logger().info("Shutting down gripper angle translator...")
    finally:
        translator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
