from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="True")

    use_python_arg = DeclareLaunchArgument("use_python", default_value="True")
    is_sim = LaunchConfiguration("is_sim")
    use_python = LaunchConfiguration("use_python")

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
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("manipulator_moveit"),
                "config",
                "manipulator.srdf",
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("manipulator_moveit"),
                "config",
                "moveit_controllers.yaml",
            )
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("manipulator_moveit"),
                "config",
                "planning_python_api.yaml",
            )
        )
        .planning_pipelines("ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Give Created Action Server Here
    remote_interface_node = Node(
        package="manipulator_remote",
        executable="task_server.py",
        condition=IfCondition(use_python),
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
        ],
    )

    return LaunchDescription(
        [
            use_python_arg,
            is_sim_arg,
            remote_interface_node,
        ]
    )
