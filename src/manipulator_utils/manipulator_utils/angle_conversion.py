#!/usr/bin/env python3

# Angle conversion service server

import rclpy
from rclpy.node import Node
from manipulator_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class AnglesConverter(Node):
    def __init__(self):
        super().__init__("angles_conversion_service_server")

        self.euler_to_quaternion_ = self.create_service(
            EulerToQuaternion, "euler_to_quaternion", self.EulerToQuaternionCallback
        )
        self.quaternion_to_euler_ = self.create_service(
            QuaternionToEuler, "quaternion_to_euler", self.QuaternionToEulerCallback
        )
        self.get_logger().info("Angle Conversion Services are Ready!")

    def EulerToQuaternionCallback(self, request, response):
        self.get_logger().info(
            "Conversion Requested of Euler Angles: roll:%f pitch:%f yaw:%f"
            % (request.roll, request.pitch, request.yaw)
        )
        quaternion = quaternion_from_euler(
            request.roll, request.pitch, request.yaw
        )  # it stores in (x, y, z, w) format
        response.x = float(quaternion[0])
        response.y = float(quaternion[1])
        response.z = float(quaternion[2])
        response.w = float(quaternion[3])

        self.get_logger().info(
            "Sending back Quaternion Angles x:%f y:%f z:%f w:%f"
            % (response.x, response.y, response.z, response.w)
        )
        return response

    def QuaternionToEulerCallback(self, request, response):
        self.get_logger().info(
            "Conversion Requested of Quaternion: x:%f y:%f z:%f w:%f"
            % (request.x, request.y, request.z, request.w)
        )
        euler = euler_from_quaternion(
            [request.x, request.y, request.z, request.w]
        )  # it returns in (roll, pitch, yaw) format
        response.roll = float(euler[0])
        response.pitch = float(euler[1])
        response.yaw = float(euler[2])

        self.get_logger().info(
            "Sending back Euler Angles roll:%f pitch:%f yaw:%f"
            % (response.roll, response.pitch, response.yaw)
        )
        return response


def main():
    rclpy.init()
    angles_converter = AnglesConverter()
    rclpy.spin(angles_converter)
    angles_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
