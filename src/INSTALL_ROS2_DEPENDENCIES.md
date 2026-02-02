# Installing ROS2 Jazz Dependencies

This guide provides step-by-step instructions for installing the necessary ROS2 system packages for the manipulator workspace.

## Prerequisites
- Ubuntu 24.04 (Jazzy supported)
- ROS2 Jazzy installed and sourced

## System Dependencies Installation

Run the following commands to install all necessary ROS2 packages:

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-tf2-ros \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-urdf \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge
```

## Alternative: Using the Automated Installer

From the workspace root, you can run the automated installer script which handles all system and Python dependencies:

```bash
sudo ./install_dependencies.sh
```

This script will:
1. Update apt package lists
2. Install rosdep (ROS dependency management tool)
3. Install all ROS2 packages for Jazzy
4. Install Python dependencies from `requirements.txt`

## Python Dependencies

After installing system packages, install Python requirements:

```bash
pip3 install -r requirements.txt
```

## Verification

To verify the installation was successful, test by sourcing ROS2 and checking for key packages:

```bash
source /opt/ros/jazzy/setup.bash
rospack find rviz2
rospack find moveit_ros_planning
```

If both commands return paths without errors, your installation is successful.

## Troubleshooting

- **Package not found error**: Ensure you ran `sudo apt update` before installing
- **ROS2 not sourced**: Add `. /opt/ros/jazzy/setup.bash` to your `.bashrc` for permanent sourcing
- **Permission denied on install_dependencies.sh**: Ensure the script is executable: `chmod +x install_dependencies.sh`

