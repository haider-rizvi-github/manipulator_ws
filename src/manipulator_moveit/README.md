# Manipulator MoveIt2 Configuration

This package contains the MoveIt2 configuration for the manipulator robot. MoveIt2 is a motion planning framework that provides an easy-to-use interface for motion planning, kinematics, object avoidance, and manipulation tasks.

## Overview

The manipulator_moveit package integrates MoveIt2 with the manipulator robot, enabling:
- **Motion Planning**: Generate collision-free trajectories for the robot
- **Kinematics**: Perform forward and inverse kinematics calculations
- **Trajectory Execution**: Execute planned trajectories on the real robot or simulation
- **Collision Detection**: Avoid obstacles in the environment
- **RViz Visualization**: Interactive planning and visualization in RViz2

## Configuration Files

- **manipulator.srdf**: Semantic Robot Description Format defining robot groups and end-effectors
- **moveit_controllers.yaml**: Controller configuration for trajectory execution
- **kinematics.yaml**: Kinematics solver configuration
- **joint_limits.yaml**: Joint limits and dynamics constraints
- **initial_positions.yaml**: Named robot configurations (home, ready positions, etc.)
- **pilz_industrial_motion_planner_planning.yaml**: Motion planning parameters
- **moveit.rviz**: RViz2 visualization configuration

## Launching MoveIt2

To launch MoveIt2 with the manipulator robot:

```bash
ros2 launch manipulator_moveit moveit.launch.py
```

This command will:
1. Start the robot state publisher
2. Load the robot description from URDF/Xacro files
3. Initialize MoveIt2 with the robot configuration
4. Launch RViz2 with pre-configured visualization settings
5. Start all necessary planning and execution components

## Usage

Once MoveIt2 is running, you can:
- Use RViz2 to visualize the robot and plan motions
- Call motion planning services via ROS2 nodes
- Execute trajectories on the robot
- Monitor joint states and robot status

## Dependencies

- ROS2 (Humble or later)
- MoveIt2
- manipulator_description package
- Gazebo (optional, for simulation)

## References

- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
