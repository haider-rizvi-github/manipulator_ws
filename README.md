# Robot Manipulator Project

This repository contains a ROS2 project for a robot manipulator with visualization capabilities. The project includes both C++ and Python examples for simple publisher subscriber and parameter, along with a complete URDF description of the robot.

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

1. Clone this repository:
```bash
git clone https://github.com/haider-rizvi-github/manipulator_ws
```

2. Build the workspace:
```bash
cd ~/manipulator_ws
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
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

### Launching Gazebo and Controllers 🔧

1) Source your workspace (if not already sourced):
```bash
source install/setup.bash
```

2) Launch Gazebo with the robot (uses `robot_description` package):
```bash
ros2 launch robot_description gazebo1.launch.py
```

3) Start the controller manager and controllers (from the `manipulator_controller` package):
```bash
# launch the controller manager and spawners (recommended)
ros2 launch manipulator_controller controller.launch.py
```

If the above launch fails (for example, due to a missing controller config file), you can spawn controllers individually:
```bash
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager
ros2 run controller_manager spawner arm_controller --controller-manager /controller_manager
ros2 run controller_manager spawner gripper_controller --controller-manager /controller_manager
```

> Tip: I noticed the package's launch references `manipulator_controllers.yaml` but the config file present is `config/manipulator_controller.yaml`. If `controller.launch.py` fails to find its YAML, either copy/rename that file or I can update the launch file to point to the correct config.

4) Useful controller and topic commands:

- List controllers and their state:
```bash
ros2 control list_controllers
```

- List active topics:
```bash
ros2 topic list
```

- Publish a command to the gripper controller (example using Float64MultiArray):
```bash
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [-1]"
```

- Echo controller topics to view messages:
```bash
ros2 topic echo /gripper_controller/state
ros2 topic echo /gripper_controller/commands
```

> Note: Controller names (e.g., `gripper_controller`) and exact CLI commands can vary depending on your ROS 2 distro and controller configuration; substitute names as needed.

### Available Examples

#### Python Examples
- Simple Publisher
- Simple Subscriber
- Simple Parameter

To run Python examples:
```bash
ros2 run arduinobot_py_examples simple_publsiher
ros2 run arduinobot_py_examples simple_subscriber
ros2 run arduinobot_py_examples simple_parameter
```
## manipulator_remote Package

The workspace includes a package named `manipulator_remote`, which provides a task-based control interface for the robot manipulator using ROS 2 Actions.

### Task Server

The file `task_server.py` implements a ROS 2 Action Server that listens on the `/task_server` action topic.  
It receives a `ManipulatorTask` goal containing a `task_number` and executes the corresponding predefined task.

Supported task numbers:
- `0`
- `1`
- `2`

Each task number triggers a different predefined behavior of the manipulator.

### Launch File

The launch file `remote_interface.launch.py` starts the task server node and enables remote task execution.

---

## Running the System with Remote Tasks

Start the system in the following order, using separate terminals.

### 1. Launch Gazebo
```bash
ros2 launch robot_description gazebo1.launch.py

2. Start Controllers
ros2 launch manipulator_controller controller.launch.py

3. Launch MoveIt
ros2 launch manipulator_moveit moveit.launch.py

4. Start the Remote Task Interface
ros2 launch manipulator_remote remote_interface.launch.py

5. Send a Task Goal
ros2 action send_goal /task_server manipulator_msgs/action/ManipulatorTask "task_number: 2"

Replace 2 with 0 or 1 to execute a different task.




## Robot Description

The robot is a 3-DOF manipulator with a gripper end-effector. The URDF description includes:
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

## Acknowledgments

- ROS2 Community
