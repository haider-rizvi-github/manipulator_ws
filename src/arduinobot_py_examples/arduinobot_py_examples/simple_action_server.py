import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from manipulator_msgs.action import Fibonacci


class SimpleActionServerNode(Node):
    def __init__(self):
        super().__init__("simple_action_server")
        self.get_logger().info("Simple Action Server Node has been started.")
        self.action_server = ActionServer(
            self, Fibonacci, "fibonacci", self.goalCallback
        )
        self.get_logger().info("Action Server 'fibonacci' is ready to receive goals.")

    def goalCallback(self, goal_handle):
        self.get_logger().info("Recieved a new goal request with order %d" % goal_handle.request.order)
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info("Feedback: partial sequence {0}".format(str(feedback_msg.partial_sequence)))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        self.get_logger().info("Goal succeeded!")
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = SimpleActionServerNode()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
