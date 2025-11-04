import rclpy
from std_msgs.msg import String
from rclpy.node import Node


class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello, world! {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main():
    rclpy.init()  # Initialize the ROS 2 Python client library
    simple_publisher = SimplePublisher()  # Create an instance of the SimplePublisher
    rclpy.spin(simple_publisher)  # Keep the node running
    simple_publisher.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shut down the ROS 2 client library


if __name__ == "__main__":
    main()
