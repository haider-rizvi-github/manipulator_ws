import rclpy
import numpy as np

from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


def move_robot():
    # Initialize MoveItPy
    manipulator = MoveItPy(node_name="moveit_py")

    # define move groups that we have defined in our MoveIt configuration
    manipulator_arm = manipulator.get_planning_component("arm_group")
    manipulator_gripper = manipulator.get_planning_component("gripper_group")

    arm_state = RobotState(manipulator.get_robot_model())
    gripper_state = RobotState(manipulator.get_robot_model())

    arm_state.set_joint_group_positions(
        "arm_group", np.array([1.57, 0.0, 0.0])
    )  # Set to initial positions for the joints base, shoulder and elbow
    gripper_state.set_joint_group_positions(
        "gripper_group", np.array([-0.7, 0.0])
    )  # Set to initial positions for the gripper joints

    # Apply the robot states to the move groups
    manipulator_arm.set_start_state_to_current_state()
    manipulator_gripper.set_start_state_to_current_state()

    manipulator_arm.set_goal_state(robot_state=arm_state)
    manipulator_gripper.set_goal_state(robot_state=gripper_state)

    # Planning the trajectory from start state to the goal state
    arm_plan_result = manipulator_arm.plan()
    gripper_plan_result = manipulator_gripper.plan()

    # checking if plan was successful
    if arm_plan_result and gripper_plan_result:
        manipulator.execute(
            arm_plan_result.trajectory, controllers=[]
        )  # Leave controllers as an empty array to use default controller
        manipulator.execute(gripper_plan_result.trajectory, controllers=[])
    else:
        get_logger("rclpy").error("One or More Planners Failed!")


def main():
    rclpy.init()  # Initialize the ROS 2 interface
    move_robot()
    rclpy.shutdown()  # Shutdown the ROS 2 interface


if __name__ == "__main__":
    main()
