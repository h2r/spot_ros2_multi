# Translates spot service into topics, so things don't block.

from threading import Lock
from typing import Callable, Any
from rclpy.node import Node


class SpotServiceTranslator(Node):
    def __init__(
        self,
        service_name: str,
        topic_name: str,
        service_type: Any,
        msg_type: Any,
        request_builder: Callable[[Any], Any],
        update_rate: float = 0.1,
    ):
        """
        Translates a ROS2 service into a topic interface.

        Args:
            service_name: Name of the service to call
            topic_name: Name of the topic to subscribe to
            service_type: Service type class (e.g., Trigger, SetBool)
            msg_type: Message type class for the topic (e.g., Bool, String)
            request_builder: Function to convert topic message to service request
            update_rate: Rate at which to check for new requests (seconds)
        """
        super().__init__(f"spot_service_translator_{topic_name.replace('/', '__')}")
        self.service_name = service_name
        self.topic_name = topic_name
        self.request_builder = request_builder

        self.last_value = None
        self.request_value = None
        self.service_in_progress = False

        # Create service client
        self.service_client = self.create_client(service_type, self.service_name)

        # Wait for service to be available
        self.get_logger().info(f"Waiting for service '{self.service_name}'...")
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service '{self.service_name}' not available, waiting...")

        self.get_logger().info(f"Service '{self.service_name}' is now available")

        # Create subscription
        self.service_lock = Lock()
        self.subscription = self.create_subscription(
            msg_type,
            self.topic_name,
            self.topic_cb,
            10
        )

        # Create timer to periodically check for new requests
        self.timer = self.create_timer(update_rate, self.update)

        self.get_logger().info(
            f"SpotServiceTranslator initialized: topic='{topic_name}' -> service='{service_name}'"
        )

    def topic_cb(self, msg):
        """Callback for topic subscription - stores the latest request."""
        with self.service_lock:
            self.request_value = msg
            self.get_logger().debug(f"Received request on topic: {msg}")

    def update(self):
        """Periodically called to check if there's a new request to send."""
        with self.service_lock:
            # Only call service if we have a new request and no service call in progress
            if (self.request_value is not None and
                self.request_value != self.last_value and
                not self.service_in_progress):

                self.service_in_progress = True
                request = self.request_builder(self.request_value)
                self.last_value = self.request_value

                # Call service asynchronously
                self.get_logger().info(f"Calling service '{self.service_name}' with request: {request}")
                future = self.service_client.call_async(request)
                future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """Callback for service response."""
        with self.service_lock:
            self.service_in_progress = False

        try:
            response = future.result()
            self.get_logger().info(f"Service response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


# Example usage
def main(args=None):
    """
    Example main function demonstrating how to use SpotServiceTranslator.

    This example translates a Bool topic into a SetBool service call.
    """
    import rclpy
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    def build_set_bool_request(msg: Bool) -> SetBool.Request:
        """Convert Bool message to SetBool service request."""
        request = SetBool.Request()
        request.data = msg.data
        return request

    rclpy.init(args=args)

    # Example: Create a translator that listens to /spot/enable topic
    # and calls /spot/claim service
    translator = SpotServiceTranslator(
        service_name="/spot/claim",
        topic_name="/spot/enable",
        service_type=SetBool,
        msg_type=Bool,
        request_builder=build_set_bool_request,
        update_rate=0.1  # Check for new requests every 100ms
    )

    try:
        rclpy.spin(translator)
    except KeyboardInterrupt:
        pass
    finally:
        translator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
