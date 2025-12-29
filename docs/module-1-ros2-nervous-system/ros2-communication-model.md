---
title: ROS 2 Communication Model
sidebar_position: 2
---

# ROS 2 Communication Model

## Overview

The ROS 2 communication model is built on the Data Distribution Service (DDS) standard, which provides a middleware layer that enables communication between different nodes in a distributed system. This communication model is fundamental to how robots coordinate their various subsystems.

## Nodes

Nodes are the fundamental building blocks of any ROS 2 system. A node is a process that performs computation and can communicate with other nodes through topics, services, actions, or parameters.

### Creating a Node

Here's how to create a basic node in Python:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Message Passing

Topics are named buses over which nodes exchange messages. They use a publish/subscribe communication pattern where publishers send messages to a topic and subscribers receive messages from the topic.

### Publishers

A publisher sends messages to a topic:

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from publisher'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
```

### Subscribers

A subscriber receives messages from a topic:

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Services

Services provide a request/response communication pattern. A service client sends a request to a service server, which processes the request and sends back a response.

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

Actions are used for goal-oriented communication with feedback. They're ideal for long-running tasks where you want to track progress and have the ability to cancel the operation.

### Action Server

```python
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Parameters

Parameters allow nodes to be configured at runtime. They provide a way to modify node behavior without recompiling.

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameter('threshold', 10.0)

        # Get parameter values
        param_value = self.get_parameter('my_param').value
        threshold_value = self.get_parameter('threshold').value

        self.get_logger().info(f'My param: {param_value}, Threshold: {threshold_value}')
```

## Communication Patterns in Humanoid Robotics

In humanoid robotics, these communication patterns enable:

- **Sensor Integration**: Multiple sensors publishing data to topics for perception systems
- **Control Coordination**: Service calls for coordinated movements between different control systems
- **Task Management**: Actions for complex behaviors like walking, grasping, or navigation
- **Configuration Management**: Parameters for tuning robot behavior based on environment or task

## Best Practices

1. **Use appropriate QoS settings** for different types of data (e.g., reliable vs. best-effort delivery)
2. **Follow naming conventions** for topics, services, and actions
3. **Handle errors gracefully** in all communication patterns
4. **Use appropriate message types** for your data
5. **Consider performance implications** when designing your communication architecture

## Agent-Controller Communication Pattern

In humanoid robotics, a common pattern is to have an agent that makes high-level decisions and a controller that executes low-level commands. Here's how they communicate:

### Example: Humanoid Movement System

The agent node receives high-level commands and translates them to low-level velocity commands:

```python
# Agent node (humanoid_agent.py)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class HumanoidAgent(Node):
    def __init__(self):
        super().__init__('humanoid_agent')
        self.command_sub = self.create_subscription(
            String, 'robot_commands', self.command_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def command_callback(self, msg):
        command = msg.data
        twist = Twist()
        if command == 'move_forward':
            twist.linear.x = 0.5  # Move forward at 0.5 m/s
        elif command == 'stop':
            twist.linear.x = 0.0  # Stop
        self.vel_pub.publish(twist)

# Controller node (humanoid_controller.py)
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.velocity_callback, 10)
        self.joint_pub = self.create_publisher(
            JointTrajectory, 'joint_trajectory', 10)

    def velocity_callback(self, msg):
        # Convert velocity command to joint trajectories
        trajectory = self.calculate_joint_trajectory(msg)
        self.joint_pub.publish(trajectory)
```

This pattern allows for clear separation of concerns: the agent focuses on decision-making while the controller handles the physics and kinematics of movement.

## Exercises

1. Create a publisher and subscriber pair that communicate sensor data
2. Implement a service that performs a simple calculation
3. Design an action for a basic robot movement task
## Summary

This chapter covered the core communication patterns in ROS 2: nodes, topics, services, actions, and parameters. You learned how to implement each pattern and saw how they work together in the agent-controller communication pattern.

## Next Steps

Continue to the next chapter to learn about representing robot structure with URDF:

[Next: Robot Structure with URDF](./robot-structure-with-urdf.md)

[Previous: Introduction to ROS 2](./introduction-to-ros2-for-physical-ai.md)