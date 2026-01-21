import rclpy
from rclpy.node import Node
from manipulator_msgs.srv import AddTwoints
import sys


class SimpleServiceClient(Node):

    def __init__(self, a, b):
        super().__init__("simple_service_client")
        self.client = self.create_client(AddTwoints, "add_two_ints")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        self.request = AddTwoints.Request()
        self.request.a = a
        self.request.b = b
        self.future_ = self.client.call_async(self.request)
        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info("Result: %d" % future.result().sum)


def main():
    rclpy.init()
    if len(sys.argv) != 3:
        print(
            "Wrong number of arguments! Usage: simple_service_client.py <int a> <int b>"
        )
        return -1

    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
