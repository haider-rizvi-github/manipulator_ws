# robot_description

![License](https://img.shields.io/badge/license-See%20LICENSE-blue) ![CI](https://img.shields.io/badge/CI-pending-lightgrey) ![ROS](https://img.shields.io/badge/ROS2--compatible-green)

**robot_description** — URDF/XACRO, meshes, RViz and Gazebo assets for the manipulator robot.

🔧 What’s included
- URDF/XACRO files: `urdf/*.xacro` (e.g., `manipulaor_ros2_control.xacro`, `manipulator_gazebo.xacro`, `manipulator.urdf.xacro`).
- Meshes in `meshes/` and Gazebo world in `world/` (`manipulator_world.sdf`).
- RViz config: `rviz/rviz.rviz`.
- Launch files: `launch/display.launch.py`, `launch/gazebo.launch.py`, and `launch/gazebo1.launch.py`.

Note
- There appears to be a minor filename typo: `urdf/manipulaor_ros2_control.xacro` — consider renaming to `manipulator_ros2_control.xacro` for consistency.

🚀 Quick start
1. Build and source the workspace:
   ```bash
   colcon build --packages-select robot_description
   source install/setup.bash
   ```
2. Visualize the robot in RViz:
   ```bash
   ros2 launch robot_description display.launch.py
   ```
3. Launch Gazebo simulation:
   ```bash
   ros2 launch robot_description gazebo.launch.py
   ```

Generating a plain URDF from a xacro (example):
```bash
xacro urdf/manipulator.urdf.xacro > manipulator.urdf
```

Integration notes
- Ensure `robot_state_publisher` and `joint_state_publisher` are available when visualizing.
- URDF/XACRO includes ros2_control tags where applicable—this pairs with `manipulator_controller`.

## Contributing

Contributions are welcome — model fixes, improved meshes, or documentation updates are very helpful.

Please:

1. Open an issue to discuss larger changes.
2. Create a branch named like `feature/...` or `fix/...`.
3. Provide model/test data and a clear description in your PR.

## License

This repository is licensed as described in the top-level `LICENSE` file. See `package.xml` for package-specific license tags.

---

If you want, I can add CI/build badges, a short URDF validation checklist, or fix the filename typo and open a PR.