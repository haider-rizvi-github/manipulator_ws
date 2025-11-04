import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class SimpleParameterNode(Node):
    def __init__(self):
        super().__init__("simple_parameter_node")

        # Declare a parameter with a default value
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "Haider!")

        # set the parameters on run time
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "simple_int_param" and param.type_ == param.Type.INTEGER:
                self.get_logger().info(f"simple_int_param changed to: {param.value}")
                result.successful = True
            elif (
                param.name == "simple_string_param" and param.type_ == param.Type.STRING
            ):
                self.get_logger().info(f"simple_string_param changed to: {param.value}")
                result.successful = True
        return result


def main():
    rclpy.init()
    node = SimpleParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
