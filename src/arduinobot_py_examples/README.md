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

# Simple Action Server - Fibonacci Action

## Overview

This document describes how to use the Simple Action Server node, which implements a ROS 2 action server that generates Fibonacci sequences. The server demonstrates the basic structure of ROS 2 action servers with real-time feedback during computation.

## Fibonacci Action Definition

The Fibonacci action is defined in `manipulator_msgs/action/Fibonacci.action`:

```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

- **Goal**: Requests a Fibonacci sequence of a given order
- **Result**: Returns the complete Fibonacci sequence
- **Feedback**: Publishes partial sequences as they are computed

## Starting the Simple Action Server

To start the simple action server node:

```bash
ros2 run arduinobot_py_examples simple_action_server
```

This will launch the action server on the `/fibonacci` action topic. You should see console output confirming:
- "Simple Action Server Node has been started."
- "Action Server 'fibonacci' is ready to receive goals."

## Monitoring Available Actions

To list all available actions in the system:

```bash
ros2 action list
```

This command will display all active action servers, including `/fibonacci`.

## Getting Action Details

To view detailed information about the Fibonacci action, including its message type:

```bash
ros2 action info /fibonacci -t
```

This will show:
- The action type: `manipulator_msgs/action/Fibonacci`
- The action server node name
- Supported clients

## Sending Goals to the Action Server

To send a goal and receive feedback:

```bash
ros2 action send_goal /fibonacci manipulator_msgs/action/Fibonacci "order: 10" -f
```

### Parameters:
- `/fibonacci`: The action server topic
- `manipulator_msgs/action/Fibonacci`: The action type
- `order: 10`: The goal (compute Fibonacci sequence up to order 10)
- `-f`: Flag to display feedback messages

### Example Output:

```
Sending goal...
Goal accepted with ID: <goal_id>

Feedback:
  partial_sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]

Result:
  sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]

Goal finished with status: SUCCEEDED
```

## How It Works

1. **Server Initialization**: The action server is created with the topic name `fibonacci` and callback `goalCallback`
2. **Goal Reception**: When a goal is received, the server extracts the order value
3. **Feedback Loop**: For each Fibonacci number generated, the server:
   - Adds the next number in the sequence
   - Publishes feedback with the partial sequence
   - Sleeps for 1 second (allowing observation of real-time feedback)
4. **Goal Completion**: After all numbers are computed, the server:
   - Marks the goal as succeeded
   - Returns the complete sequence as the result

## Complete Workflow Example

In one terminal, start the server:
```bash
ros2 run arduinobot_py_examples simple_action_server
```

In another terminal, check available actions:
```bash
ros2 action list
```

Get action details:
```bash
ros2 action info /fibonacci -t
```

Send a goal and observe feedback:
```bash
ros2 action send_goal /fibonacci manipulator_msgs/action/Fibonacci "order: 10" -f
```

## Simple Action Client

An action client is provided that automatically sends goals to the action server and processes responses programmatically.

### Running the Simple Action Client

To run the action client node that automatically requests a Fibonacci sequence and handles responses:

```bash
ros2 run arduinobot_py_examples simple_action_client
```

### What the Action Client Does

The `SimpleActionClientNode` performs the following operations:

1. **Connects to the Action Server**: Waits for the `/fibonacci` action server to be available
2. **Sends a Goal**: Sends a goal request with `order: 10` to compute the first 10 Fibonacci numbers
3. **Handles Responses**: 
   - Receives a confirmation that the goal was accepted
   - Receives real-time feedback as each Fibonacci number is computed
   - Displays the feedback in the console
4. **Receives Results**: Gets the final complete Fibonacci sequence when the computation is done
5. **Shuts Down**: Exits gracefully after receiving the result

### Example Output:

```
Goal accepted :)
Received feedback: [0, 1]
Received feedback: [0, 1, 1]
Received feedback: [0, 1, 1, 2]
Received feedback: [0, 1, 1, 2, 3]
...
Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
```

### How It Differs from `ros2 action send_goal`

- **Programmatic**: The action client is a ROS 2 node that can be integrated into larger applications
- **Automated**: It automatically handles goal submission and result retrieval without manual command-line input
- **Real-time Processing**: Responses and feedback are processed through callbacks
- **Node-based**: Can be modified to send different goals or perform custom logic based on feedback

## Complete Workflow with Action Client

### Terminal 1 - Start the Action Server:
```bash
ros2 run arduinobot_py_examples simple_action_server
```

### Terminal 2 - Run the Action Client:
```bash
ros2 run arduinobot_py_examples simple_action_client
```

The client will automatically connect, send a goal, display feedback as it's received, and exit after getting the result.

## Notes

- Each Fibonacci computation includes a 1-second delay between steps, allowing observation of real-time feedback
- The action server will continue running and accept multiple goals sequentially
- The action client automatically sends a hardcoded goal with `order: 10` (can be modified in the source code)
- Press `Ctrl+C` to stop either the server or client
- Feedback is automatically displayed by the action client through callbacks
- Multiple clients can connect to the same action server
