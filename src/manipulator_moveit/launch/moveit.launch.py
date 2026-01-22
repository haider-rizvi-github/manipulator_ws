"""
This launch file configures and launches the MoveIt move_group node for the manipulator robot.

It sets up the necessary configurations for MoveIt, including robot description, semantic description,
trajectory execution, and simulation parameters.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """
    Generate the launch description for the MoveIt move_group node.

    This function creates the necessary launch arguments and nodes to start the MoveIt move_group
    for the manipulator robot in either simulation or real robot mode.

    Returns:
        LaunchDescription: The launch description containing the move_group node and arguments.
    """
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Flag to indicate simulation or real robot",
    )

    is_sim = LaunchConfiguration("is_sim")

    # Build the MoveIt configuration using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("manipulator", package_name="manipulator_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("robot_description"),
                "urdf",
                "manipulator.urdf.xacro",
            )
        )
        .robot_description_semantic(file_path="config/manipulator.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines("ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Create the move_group node with the configured parameters
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz configuration file path
    rviz_config = os.path.join(
        get_package_share_directory("manipulator_moveit"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Return the launch description with the is_sim argument and move_group node
    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node,
            rviz_node,
        ]
    )
