import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")
        self.subscription = self.create_subscription(
            String,
            "chatter",
            self.listener_callback,
            10,
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main():
    rclpy.init()  # Initialize the ROS 2 Python client library
    simple_subscriber = SimpleSubscriber()  # Create an instance of the SimpleSubscriber
    rclpy.spin(simple_subscriber)  # Keep the node running
    simple_subscriber.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shut down the ROS 2 client library


if __name__ == "__main__":
    main()
