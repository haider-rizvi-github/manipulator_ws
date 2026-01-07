import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description_dir = get_package_share_directory("robot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            robot_description_dir, "urdf", "manipulator.urdf.xacro"
        ),
        description="Absolute path to robot urdf file",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    joint_state_pub = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(robot_description_dir).parent.resolve())],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "my_robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.1",
        ],
    )

    bridge_params = os.path.join(
        get_package_share_directory("manipulator_controller"),
        "config",
        "ros_gz_bridge.yaml",
    )
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
    )

    rviz_file = "rviz.rviz"
    rviz_path = PathJoinSubstitution(
        [get_package_share_directory("robot_description"), rviz_file]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
    )
    return LaunchDescription(
        [
            model_arg,
            gazebo_resource_path,
            robot_state_publisher_node,
            joint_state_pub,
            gazebo,
            gz_spawn_entity,
            gz_ros2_bridge,
            rviz2_node,
        ]
    )
