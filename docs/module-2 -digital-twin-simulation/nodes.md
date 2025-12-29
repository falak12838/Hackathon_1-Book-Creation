---
title: Understanding ROS 2 Nodes
sidebar_position: 4
---

# Understanding ROS 2 Nodes

## What is a Node?

A node in ROS 2 is an executable that uses ROS 2 client libraries to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system, where each node typically performs a specific task or function within the larger robotic system.

In humanoid robotics, nodes can represent different subsystems such as:
- Perception nodes (processing camera, LIDAR, IMU data)
- Control nodes (managing joint positions, balance)
- Planning nodes (path planning, motion planning)
- Interaction nodes (speech, gesture recognition)

## Node Architecture in Humanoid Robots

Humanoid robots typically have a distributed architecture with multiple nodes running on different computational units throughout the robot's body. This allows for:

- **Modularity**: Each function can be developed and tested independently
- **Scalability**: Additional nodes can be added as needed
- **Fault Tolerance**: Failure of one node doesn't necessarily affect others
- **Performance**: Computationally intensive tasks can be distributed

## Creating Nodes in rclpy

The Python client library for ROS 2 (rclpy) provides the tools to create nodes. Here's a more detailed example of a humanoid-specific node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class HumanoidBalanceNode(Node):
    def __init__(self):
        super().__init__('humanoid_balance_node')

        # Subscribe to sensor data
        self.imu_subscriber = self.create_subscription(
            sensor_msgs.msg.Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # Publish balance corrections
        self.correction_publisher = self.create_publisher(
            Float64MultiArray,
            'balance_corrections',
            10
        )

        # Timer for balance control loop
        self.control_timer = self.create_timer(0.01, self.balance_control_loop)

        self.get_logger().info('Humanoid Balance Node initialized')

    def imu_callback(self, msg):
        # Process IMU data for balance calculations
        self.current_orientation = msg.orientation
        self.current_angular_velocity = msg.angular_velocity

    def balance_control_loop(self):
        # Implement balance control algorithm
        corrections = self.calculate_balance_corrections()
        if corrections is not None:
            msg = Float64MultiArray(data=corrections)
            self.correction_publisher.publish(msg)

    def calculate_balance_corrections(self):
        # Placeholder for balance control algorithm
        # This would implement the actual balance control logic
        return [0.0, 0.0, 0.0]  # Example corrections

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidBalanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Parameters and Configuration

Nodes can be configured using parameters, which allows for runtime configuration without recompiling:

```python
class ConfigurableHumanoidNode(Node):
    def __init__(self):
        super().__init__('configurable_humanoid_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_humanoid')
        self.declare_parameter('control_frequency', 100)
        self.declare_parameter('safety_threshold', 0.5)

        # Access parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
```

## Best Practices for Node Development

1. **Single Responsibility**: Each node should have a clear, single purpose
2. **Error Handling**: Implement proper error handling and graceful degradation
3. **Resource Management**: Properly clean up resources when the node shuts down
4. **Logging**: Use appropriate logging levels for debugging and monitoring
5. **Testing**: Design nodes to be testable in isolation