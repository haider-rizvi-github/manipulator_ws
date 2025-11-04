# Robot Manipulator Project

This repository contains a ROS2 project for a robot manipulator with visualization capabilities. The project includes both C++ and Python examples, along with a complete URDF description of the robot.

## Project Structure

```
├── arduinobot_cpp_examples/    # C++ implementation examples
├── arduinobot_py_examples/     # Python implementation examples
└── robot_description/          # Robot URDF and mesh files
```

## Prerequisites

Before running this project, ensure you have the following dependencies installed:

### System Requirements
- Ubuntu 22.04 or later
- ROS2 Jazzy
- Python 3.8+

### ROS2 Dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2
```

## Installation

1. Create a ROS2 workspace:
```bash
mkdir -p ~/manipulator_ws/src
cd ~/manipulator_ws/src
```

2. Clone this repository:
```bash
git clone <your-repository-url>
```

3. Build the workspace:
```bash
cd ~/manipulator_ws
colcon build
```

4. Source the workspace:
```bash
source ~/manipulator_ws/install/setup.bash
```

## Usage

### Launching the Robot Visualization

To visualize the robot in RViz:
```bash
ros2 launch robot_description display.launch.py
```

This will launch:
- RViz for visualization
- Joint State Publisher GUI for controlling joint positions
- Robot State Publisher for publishing transforms

### Available Examples

#### Python Examples
- Simple Publisher
- Simple Subscriber
- Parameter Handling

To run Python examples:
```bash
ros2 run arduinobot_py_examples <example_name>
```

#### C++ Examples
The C++ examples demonstrate basic ROS2 concepts and robot control.

To run C++ examples:
```bash
ros2 run arduinobot_cpp_examples <example_name>
```

## Robot Description

The robot is a 5-DOF manipulator with a gripper end-effector. The URDF description includes:
- Base link and plate
- Forward drive arm
- Horizontal arm
- Claw support
- Gripper fingers (left and right)

Joint configuration:
- Base rotation (±90 degrees)
- Forward arm rotation (±90 degrees)
- Horizontal arm rotation (±90 degrees)
- Gripper control

## Contributing

Feel free to open issues and pull requests for any improvements or bug fixes.

## License

[Add your license information here]

## Acknowledgments

- ROS2 Community
- [Add any other acknowledgments]