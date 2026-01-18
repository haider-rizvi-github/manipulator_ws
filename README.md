# Manipulator Robot Simulation

This project contains the ROS 2 packages to simulate a manipulator robot in Gazebo. It includes the robot description (URDF), controllers, and a bringup launch file to start the simulation.

## Dependencies

*   ROS 2 Humble Hawksbill
*   Gazebo (Ignition Gazebo Fortress)
*   `ros-humble-ros-gz` package

## Build Instructions

1.  Source your ROS 2 installation:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

2.  Build the workspace:
    ```bash
    colcon build
    ```

## Running the Simulation

1.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

2.  Launch the simulation:
    ```bash
    ros2 launch manipulator_bringup bringup.launch.py
    ```
    This will start Gazebo, spawn the robot, and launch the necessary controllers.

## Controlling the Robot

You can control the robot's arm from the terminal by publishing a `JointTrajectory` message to the `/arm_controller/joint_trajectory` topic.

Here is a demo command to move the arm to a new position:

Test the Arm Joints
```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  "header": {
    "frame_id": ""
  },
  "joint_names": [
    "basement_to_base_plate",
    "base_plate_to_forward_drive_arm",
    "forward_drive_arm_to_horizontal_arm"
  ],
  "points": [
    {
      "positions": [-0.5, -0.5, -0.5],
      "velocities": [],
      "accelerations": [],
      "effort": [],
      "time_from_start": {
        "sec": 2,
        "nanosec": 0
      }
    }
  ]
}'
```
Test the Gripper
```bash
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  "header": {
    "frame_id": ""
  },
  "joint_names": [
    "claw_support_to_gripper_finger_left"
  ],
  "points": [
    {
      "positions": [0.7], 
      "velocities": [],
      "accelerations": [],
      "effort": [],
      "time_from_start": {
        "sec": 2,
        "nanosec": 0
      }
    }
  ]
}'
```