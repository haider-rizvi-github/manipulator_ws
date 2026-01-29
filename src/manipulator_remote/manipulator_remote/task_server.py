#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from manipulator_msgs.action import ManipulatorTask  # adding the action msg
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Server is getting started.")
        self.action_server = ActionServer(
            self, ManipulatorTask, "task_server", self.goalCallback
        )
        self.get_logger().info("ACTION_SERVER:task_server is ready to receive goals.")

        # Initializing moveit API
        self.manipulator = MoveItPy(node_name="moveit_py")

        # Accessing planning groups we created in config
        self.manipulator_arm = self.manipulator.get_planning_component("arm_group")
        self.manipulator_gripper = self.manipulator.get_planning_component(
            "gripper_group"
        )

    def goalCallback(self, goal_handle):
        self.get_logger().info(
            "Recieved a new goal request with task_number %d"
            % goal_handle.request.task_number
        )

        # reading current state of robot
        arm_state = RobotState(self.manipulator.get_robot_model())
        gripper_state = RobotState(self.manipulator.get_robot_model())

        # Now changing orientation of robot to desire state
        arm_joint_goal = []
        gripper_joint_goal = []

        # Note: we want to set the orientation fepending upon task_number
        if goal_handle.request.task_number == 0:
            arm_joint_goal = np.array([0.0, 0.0, 0.0])
            gripper_joint_goal = np.array([-0.7, 0.7])  # 0.7 Radian = 40.107 Degree
        elif goal_handle.request.task_number == 1:  # Picking up Position
            arm_joint_goal = np.array([-1.14, -0.6, -0.07])
            gripper_joint_goal = np.array([0.0, 0.0])
        elif goal_handle.request.task_number == 2:  # Home Position
            arm_joint_goal = np.array([-1.57, 0.0, -0.9])
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().error("Invalid Task Number!")
            return

        # Set the target position depending upon the task_number
        arm_state.set_joint_group_positions("arm_group", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper_group", gripper_joint_goal)

        # Updating the start states of moveGroup
        self.manipulator_arm.set_start_state_to_current_state()
        self.manipulator_gripper.set_start_state_to_current_state()

        # Updating the Goal states of moveGroup
        self.manipulator_arm.set_goal_state(robot_state=arm_state)
        self.manipulator_gripper.set_goal_state(robot_state=gripper_state)

        # Planning a trajectory with plan function
        arm_plan_result = self.manipulator_arm.plan()
        gripper_plan_result = self.manipulator_gripper.plan()

        # Check if planner is valid
        if arm_plan_result and gripper_plan_result:
            self.manipulator.execute(arm_plan_result.trajectory, controllers=[])
            self.manipulator.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().info("One or more planners failed")

        # return the result of action server
        goal_handle.succeed()
        result = ManipulatorTask.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)
    task_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
