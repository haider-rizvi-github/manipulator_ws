from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import SetEnvironmentVariable, GroupAction
from launch.substitutions import EnvironmentVariable, TextSubstitution


def generate_launch_description():

    # we add gazebo launch file here
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_description"),
            "launch",
            "gazebo1.launch.py",
        )
    )

    # we add ros2_control launch file here
    ros2control = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("manipulator_controller"),
            "launch",
            "controller.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    # we add moveit launch file here

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("manipulator_moveit"),
            "launch",
            "moveit.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    # we add remote_interface launch file here
    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("manipulator_remote"),
            "launch",
            "remote_interface.launch.py",
        )
    )

    with_env = GroupAction(
        [
            SetEnvironmentVariable(
                name="PATH",
                value=[
                    TextSubstitution(text="/home/syed/local/openssl-3.0.9/bin:"),
                    EnvironmentVariable(
                        name="PATH", default_value=TextSubstitution(text="")
                    ),
                ],
            ),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=[
                    TextSubstitution(text="/home/syed/local/openssl-3.0.9/lib64:"),
                    EnvironmentVariable(
                        name="LD_LIBRARY_PATH", default_value=TextSubstitution(text="")
                    ),
                ],
            ),
            SetEnvironmentVariable(
                name="SSL_CERT_FILE",
                value=TextSubstitution(text="/etc/ssl/certs/ca-certificates.crt"),
            ),
            SetEnvironmentVariable(
                name="SSL_CERT_DIR", value=TextSubstitution(text="/etc/ssl/certs")
            ),
            gazebo,
            ros2control,
            moveit,
            remote_interface,
        ]
    )

    return LaunchDescription([with_env])

    return LaunchDescription([gazebo, ros2control, moveit, remote_interface])
    # return LaunchDescription([gazebo, ros2control, moveit, remote_interface])
