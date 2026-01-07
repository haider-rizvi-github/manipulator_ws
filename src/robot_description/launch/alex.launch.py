import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro

# define the package name
packageName = "robot_description"

# Define controller package name
ControllerPackageName = "manipulator_controller"

# relative path of the xacro file with respect to the package path
xacroRelativePath = "urdf/manipulator.urdf.xacro"

# Rviz config file path with respect to the package path
rvizRelativePath = "rviz/rviz.rviz"

# relative path of the ros2_control configuration file with respect to the package folder

ros2controlRelativePath = "config/ros2_control.yaml"


def generate_launch_description():
    # absoulute package path
    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(
        packageName
    )

    # absolute path of the controller file
    ControllerpkgPath = launch_ros.substitutions.FindPackageShare(
        package=ControllerPackageName
    ).find(ControllerPackageName)

    # absolute xacro model path
    xacroModelPath = os.path.join(pkgPath, xacroRelativePath)
    # absolute rviz config file path
    rvizConfigPath = os.path.join(pkgPath, rvizRelativePath)

    # absolute ros2_control config file path
    ros2controlPath = os.path.join(ControllerpkgPath, ros2controlRelativePath)
    # here, for verification, print the xacro model path
    print("Xacro model path: ", xacroModelPath)

    # get the robot description from the xacro file
    robot_desc = xacro.process_file(xacroModelPath).toxml()

    # define a parameter with the robot xacro description
    robot_description = {"robot_description": robot_desc}

    # Declare arguments
    Declared_arguments = []
    Declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="gui", default_value="true", description="Start the Rviz2 GUI"
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # for starting Gazebo
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                launch_ros.substitutions.FindPackageShare("ros_gz_sim"),
                "/launch/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", "-r -v 3 empty.sdf")],
        condition=launch.conditions.IfCondition(gui),
    )

    gazebo_headless = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                launch_ros.substitutions.FindPackageShare("ros_gz_sim"),
                "/launch/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", ["--headless -s -r -v 3 empty.sdf"])],
        condition=launch.conditions.UnlessCondition(gui),
    )

    # Gazebo bridge
    gazebo_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/manipulator/joint_state@sensor_msgs/msg/JointState[gz.msgs.msg.ModelJointState",
            "/model/manipulator/pose@geometry_msgs/msg/PoseStamped[gz.msgs.msg.Pose",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.msg.Clock",
        ],
        output="screen",
    )

    gz_spawn_entity = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "manipulator",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.0",
        ],
    )

    # robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Rviz2 node
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizConfigPath],
    )

    # ros2_control node
    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2controlPath],
        output="both",
    )

    # joint state broadcaster
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # forward position controller
    robot_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", ros2controlPath],
    )

    nodeList = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        gz_spawn_entity,
        robot_state_publisher_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return launch.LaunchDescription(Declared_arguments + nodeList)
