# Manipulator Utils

This package provides utility nodes for the manipulator robot, including angle conversion services.

## Angle Conversion Service

The `angle_conversion.py` node provides ROS2 services for converting between Euler angles and quaternions.

### Services Provided

1. **euler_to_quaternion**: Converts Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w).
2. **quaternion_to_euler**: Converts quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).

### How It Works

The service uses the `tf_transformations` library to perform the conversions:
- `quaternion_from_euler(roll, pitch, yaw)` for Euler to Quaternion.
- `euler_from_quaternion([x, y, z, w])` for Quaternion to Euler.

### Running the Node

To run the angle conversion service node:

```bash
ros2 run manipulator_utils angle_conversion.py
```

### Example Usage

#### Convert Euler to Quaternion

```bash
ros2 service call /euler_to_quaternion manipulator_msgs/srv/EulerToQuaternion "roll: -0.5
pitch: 0.0
yaw: -1.5"
```

Expected response will contain the quaternion components x, y, z, w.

#### Convert Quaternion to Euler

```bash
ros2 service call /quaternion_to_euler manipulator_msgs/srv/QuaternionToEuler "x: 0.0
y: 0.0
z: -0.707
w: 0.707"
```

Expected response will contain the Euler angles roll, pitch, yaw.

You can call either service while the node is running. The node logs the conversion requests and responses for debugging.

### Dependencies

- rclpy
- manipulator_msgs
- tf_transformations
- python3 library transforms3d