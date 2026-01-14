# manipulator_controller

![License](https://img.shields.io/badge/license-See%20LICENSE-blue) ![CI](https://img.shields.io/badge/CI-pending-lightgrey) ![ROS](https://img.shields.io/badge/ROS2--compatible-green)

**manipulator_controller** — ROS 2 controller package for the manipulator robot.

🔧 Features
- Launchable controller stack (`launch/controller.launch.py`).
- Centralized controller parameters: `config/manipulator_controller.yaml`.
- Sources in `src/` and headers in `include/`.

🚀 Quick start
1. From your workspace root:
   ```bash
   colcon build --packages-select manipulator_controller
   source install/setup.bash
   ```
2. Launch the controller(s):
   ```bash
   ros2 launch manipulator_controller controller.launch.py
   ```

Notes
- Start `robot_description` (visualization/simulation) before launching the controllers so they can attach to the robot state and hardware/sim interface.
- Use `ros2 topic list`, `ros2 topic echo <topic>`, and `ros2 node info <node>` to inspect runtime behavior.

Configuration
- Edit `config/manipulator_controller.yaml` to tune controller gains, topics, timeouts, and other parameters.

Dependencies
- See `package.xml` for full list; typical dependencies include `ros2_control`, `controller_manager`, and core ROS 2 packages (xacro, robot_state_publisher, etc.).

## Contributing

Contributions are welcome! Please follow these steps:

1. Open an issue to discuss significant changes.
2. Create a branch with a descriptive name: `feature/...` or `fix/...`.
3. Add or update tests and follow the project's coding style.
4. Submit a pull request and reference any related issues.

## License

This project is licensed as described in the top-level `LICENSE` file. See `package.xml` for package-specific license tags.

---

If you'd like, I can add badges (license / build / ROS distro) and a short example showing how to pair this with the `robot_description` package.