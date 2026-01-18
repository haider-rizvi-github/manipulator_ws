import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_description_dir = get_package_share_directory("robot_description")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(robot_description_dir).parent.resolve())],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])],
    )

    return LaunchDescription(
        [
            gazebo_resource_path,
            gazebo,
        ]
    )

