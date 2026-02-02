# Manipulator Workspace

This workspace contains ROS2 packages for controlling and simulating an Arduino-based robotic manipulator with MoveIt integration and Gazebo simulation.

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

This installs `flask`, `flask-ask-sdk`, `ask-sdk-core`, `ask-sdk-model`, `numpy`.

**NGROK**
You should follow the steps to install NGROK:
[text](https://dashboard.ngrok.com/get-started/setup/linux)


## Package Overview

- **robot_description**: URDF/XACRO files and Gazebo simulation files for the manipulator
- **manipulator_controller**: Controller configuration and launch files
- **manipulator_moveit**: MoveIt motion planning configuration and launch files
- **manipulator_msgs**: Custom ROS2 message and action definitions
- **manipulator_utils**: Utility functions for angle conversions
- **manipulator_bringup**: Launches all the launch files from the single launch file
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

## NOTE
Do not forget to add Motion Planning and select the joint group you want to move in RVIZ2.


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

[text](https://developer.amazon.com/alexa/console/ask/test/amzn1.ask.skill.8303b30e-fb9b-4d1f-9ec4-8cb57f872915/development/en_US/)

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



- **Future Work**:

  -Integrate a camera with gazebo as it already have a camera urdf.
  -Apply object detection algorithm and make the robot work accordingly.
  -Build a suitable world for the manipulator.

