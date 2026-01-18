import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_description"),
                "launch",
                "gazebo.launch.py",
            )
        ),
    )

    gz_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "manipulator",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.1",
        ],
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("manipulator_controller"),
                "launch",
                "controller.launch.py",
            )
        ),
    )

    bridge_params = os.path.join(
        get_package_share_directory("manipulator_controller"),
        "config",
        "ros_gz_bridge.yaml",
    )
    gz_ros2_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
    )


    return LaunchDescription(
        [
            model_arg,
            robot_state_publisher_node,
            gazebo_launch,
            gz_spawn_entity_node,
            controller_launch,
            gz_ros2_bridge_node,
        ]
    )
