# Arduinobot Python Examples

This package contains Python examples for ROS2, demonstrating basic ROS2 concepts such as publishers, subscribers, parameters, and services.

## Simple Service Server

The `simple_service_server.py` file implements a ROS2 service server that adds two integers.

### Running the Service Server

To run the service server:

```bash
ros2 run arduinobot_py_examples simple_service_server
```

### Requesting from the Service Server

To call the service with specific values (e.g., a=7, b=5):

```bash
ros2 service call /add_two_ints manipulator_msgs/srv/AddTwoints "{a: 7, b: 5}"
```

### Inspecting the Service

- List all nodes:
  ```bash
  ros2 node list
  ```

- Get info about the service server node:
  ```bash
  ros2 service info /simple_service_server
  ```

- Get info about the add_two_ints service:
  ```bash
  ros2 service info /add_two_ints
  ```

- Call the service without parameters (will prompt for input):
  ```bash
  ros2 service call /add_two_ints manipulator_msgs/srv/AddTwoints
  ```

## Simple Service Client

The `simple_service_client.py` file implements a ROS2 service client that sends a request to add two integers to the `add_two_ints` service.

### Running the Service Client

To run the service client with specific values (e.g., a=7, b=5):

```bash
ros2 run arduinobot_py_examples simple_service_client 7 5
```

**Note**: The service server must be running for the client to work. Start the server first with `ros2 run arduinobot_py_examples simple_service_server`.