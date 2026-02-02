# Manipulator Workspace

This workspace contains ROS2 packages for controlling and simulating an Arduino-based robotic manipulator with MoveIt integration and Gazebo simulation.

## Package Overview

- **robot_description**: URDF/XACRO files and Gazebo simulation files for the manipulator
- **manipulator_controller**: Controller configuration and launch files
- **manipulator_moveit**: MoveIt motion planning configuration and launch files
- **manipulator_msgs**: Custom ROS2 message and action definitions
- **manipulator_utils**: Utility functions for angle conversions
- **
- **arduinobot_py_examples**: Python example nodes and scripts

## Launching the Manipulator with MoveIt Interface

To run the complete manipulator system with MoveIt motion planning interface, you need to launch multiple components in separate terminals:

### Step 1: Launch Gazebo Simulation
```bash
ros2 launch robot_description gazebo1.launch.py
```
**What happens:**
- Loads the manipulator robot description (URDF/XACRO files)
- Starts the Gazebo simulator
- Spawns the manipulator robot in the simulated environment
- Initializes the robot with default joint positions

### Step 2: Launch Controller
```bash
ros2 launch manipulator_controller controller.launch.py
```
**What happens:**
- Starts the ros2_control controller manager
- Loads and activates the joint trajectory controller
- Enables communication between ROS2 and the simulated robot in Gazebo
- Allows you to send joint commands to the manipulator

### Step 3: Launch MoveIt
```bash
ros2 launch manipulator_moveit moveit.launch.py
```
**What happens:**
- Initializes the MoveIt motion planning framework
- Loads the manipulator's kinematics solver
- Starts the planning scene monitor
- Launches RViz with the manipulator model for visualization
- Enables motion planning and trajectory execution capabilities

### Step 4: Launch Simple MoveIt Interface
```bash
ros2 launch arduinobot_py_examples simple_moveit_interface.launch.py
```
**What happens:**
- Starts the Python node that provides a simple interface to MoveIt
- Enables you to:
  - Plan motions to specific poses or joint configurations
  - Execute planned trajectories on the manipulator
  - Interact with the manipulator through a user-friendly interface
  - Visualize motion plans in RViz before execution
- This node acts as a bridge between user commands and the MoveIt planning system

## Complete Launch Sequence

For a quick setup, open 4 terminals and run these commands in order:

**Terminal 1:**
```bash
ros2 launch robot_description gazebo1.launch.py
```

**Terminal 2:**
```bash
ros2 launch manipulator_controller controller.launch.py
```

**Terminal 3:**
```bash
ros2 launch manipulator_moveit moveit.launch.py
```

**Terminal 4:**
```bash
ros2 launch arduinobot_py_examples simple_moveit_interface.launch.py
```

## System Architecture

The launch sequence establishes the following architecture:

1. **Gazebo** (robot_description) - Provides physics simulation and sensor feedback
2. **ros2_control** (manipulator_controller) - Translates high-level commands into joint commands
3. **MoveIt** (manipulator_moveit) - Handles motion planning and collision detection
4. **Simple Interface** (arduinobot_py_examples) - Provides user-friendly access to MoveIt capabilities

## Dependencies

Ensure you have installed:
- ROS2 (matching your distribution)
- Gazebo
- MoveIt2
- ros2_control
- ros2_controllers

## Install Dependencies

For detailed ROS2 Jazzy dependency installation instructions, see [INSTALL_ROS2_DEPENDENCIES.md](INSTALL_ROS2_DEPENDENCIES.md).

### Quick Install (Automated)

From the workspace root, run the installer script:

```bash
sudo ./install_dependencies.sh
```

This installs:
- System & ROS2 packages (xacro, joint_state_publisher_gui, robot_state_publisher, rviz2, MoveIt, tf2, ros2_control, etc.)
- Python dependencies (Flask, Ask SDK, numpy, requests, aiohttp, boto3, etc.)

### Manual Install

**ROS2 System Packages:**

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros \
    ros-jazzy-tf2-ros \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-urdf \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge
```

**Python Requirements:**

From the workspace root:

```bash
pip3 install -r requirements.txt
```

This installs `flask`, `flask-ask-sdk`, `ask-sdk-core`, `ask-sdk-model`, `numpy`, `requests`, `aiohttp`, `boto3`, and test tools.


## Notes

- All terminals should be kept running for the complete system to function
- RViz will open automatically with the MoveIt launch, showing real-time visualization
- Gazebo runs the physics simulation and displays the robot
- The simple_moveit_interface provides Python APIs for programmatic control

## Bringup & Alexa Integration

- **Overview**: These steps show how to start the manipulator bringup and connect the Alexa skill (via an ngrok HTTP tunnel) so you can control the robot using voice commands.

- **Prerequisites**: Ensure your ROS2 workspace is built and sourced, `ngrok` is installed and available on your PATH, and you have created/configured an Alexa skill in the Alexa Developer Console.

- **Start ngrok (required before bringup)**: In a separate terminal, run:

```bash
ngrok http 5000
```

This opens a public HTTPS tunnel forwarding to local port `5000` required by the Alexa/skill webhook.

- **Launch the manipulator bringup**: In another terminal (after sourcing your workspace), run:

```bash
ros2 launch manipulator_bringup simulated_robot.launch.py
```

- **Alexa Developer Console (Testing)**: Open the Alexa Developer Console, select your skill, then go to the "Test" tab and enable skill testing by switching the test mode to **Development**. Use the test console or an Alexa device linked to the same developer account to send sample utterances.

- **Sample voice commands and Alexa responses**:

  - **Command:** my robot
    - **Alexa response:** hi how can i help you

  - **Command:** wake up
    - **Alexa response:** Robot is waking up

  - **Command:** pick pen
    - **Alexa response:** Picking up the item

  - **Command:** sleep
    - **Alexa response:** Robot is going to sleep.

- **Notes**:
  - Start `ngrok` before launching the bringup so the skill endpoint is reachable.
  - Keep the terminal running `ngrok` and the bringup launch open while testing the skill.
  - If your skill uses a different port, replace `5000` in the `ngrok` command accordingly.
# Manipulator Workspace

This workspace contains ROS2 packages for controlling and simulating an Arduino-based robotic manipulator with MoveIt integration and Gazebo simulation.

## Package Overview

- **robot_description**: URDF/XACRO files and Gazebo simulation files for the manipulator
- **manipulator_controller**: Controller configuration and launch files
- **manipulator_moveit**: MoveIt motion planning configuration and launch files
- **manipulator_msgs**: Custom ROS2 message and action definitions
- **manipulator_utils**: Utility functions for angle conversions
- **arduinobot_cpp_examples**: C++ example nodes
- **arduinobot_py_examples**: Python example nodes and scripts

## Launching the Manipulator with MoveIt Interface

To run the complete manipulator system with MoveIt motion planning interface, you need to launch multiple components in separate terminals:

### Step 1: Launch Gazebo Simulation
```bash
ros2 launch robot_description gazebo1.launch.py
```
**What happens:**
- Loads the manipulator robot description (URDF/XACRO files)
- Starts the Gazebo simulator
- Spawns the manipulator robot in the simulated environment
- Initializes the robot with default joint positions

### Step 2: Launch Controller
```bash
ros2 launch manipulator_controller controller.launch.py
```
**What happens:**
- Starts the ros2_control controller manager
- Loads and activates the joint trajectory controller
- Enables communication between ROS2 and the simulated robot in Gazebo
- Allows you to send joint commands to the manipulator

### Step 3: Launch MoveIt
```bash
ros2 launch manipulator_moveit moveit.launch.py
```
**What happens:**
- Initializes the MoveIt motion planning framework
- Loads the manipulator's kinematics solver
- Starts the planning scene monitor
- Launches RViz with the manipulator model for visualization
- Enables motion planning and trajectory execution capabilities

### Step 4: Launch Simple MoveIt Interface
```bash
ros2 launch arduinobot_py_examples simple_moveit_interface.launch.py
```
**What happens:**
- Starts the Python node that provides a simple interface to MoveIt
- Enables you to:
  - Plan motions to specific poses or joint configurations
  - Execute planned trajectories on the manipulator
  - Interact with the manipulator through a user-friendly interface
  - Visualize motion plans in RViz before execution
- This node acts as a bridge between user commands and the MoveIt planning system

## Complete Launch Sequence

For a quick setup, open 4 terminals and run these commands in order:

**Terminal 1:**
```bash
ros2 launch robot_description gazebo1.launch.py
```

**Terminal 2:**
```bash
ros2 launch manipulator_controller controller.launch.py
```

**Terminal 3:**
```bash
ros2 launch manipulator_moveit moveit.launch.py
```

**Terminal 4:**
```bash
ros2 launch arduinobot_py_examples simple_moveit_interface.launch.py
```

## System Architecture

The launch sequence establishes the following architecture:

1. **Gazebo** (robot_description) - Provides physics simulation and sensor feedback
2. **ros2_control** (manipulator_controller) - Translates high-level commands into joint commands
3. **MoveIt** (manipulator_moveit) - Handles motion planning and collision detection
4. **Simple Interface** (arduinobot_py_examples) - Provides user-friendly access to MoveIt capabilities

## Dependencies

Ensure you have installed:
- ROS2 (matching your distribution)
- Gazebo
- MoveIt2
- ros2_control
- ros2_controllers

## Notes

- All terminals should be kept running for the complete system to function
- RViz will open automatically with the MoveIt launch, showing real-time visualization
- Gazebo runs the physics simulation and displays the robot
- The simple_moveit_interface provides Python APIs for programmatic control
