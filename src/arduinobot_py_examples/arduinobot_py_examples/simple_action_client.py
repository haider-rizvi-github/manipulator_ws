import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from manipulator_msgs.action import Fibonacci


class SimpleActionClientNode(Node):
    def __init__(self):
        super().__init__("fibonacci_action_client")
        self._action_client = ActionClient(self, Fibonacci, "fibonacci")

        self._action_client.wait_for_server()
        self.goal = Fibonacci.Goal()
        self.goal.order = 10
        self.future = self._action_client.send_goal_async(
            self.goal, feedback_callback=self.feedbackCallback
        )
        self.future.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.sequence))
        rclpy.shutdown()

    def feedbackCallback(self, feedback_msg):
        self.get_logger().info(
            "Received feedback: {0}".format(feedback_msg.feedback.partial_sequence)
        )


def main(args=None):
    rclpy.init(args=args)

    action_client = SimpleActionClientNode()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
