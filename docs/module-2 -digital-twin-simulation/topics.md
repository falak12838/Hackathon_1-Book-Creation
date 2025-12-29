---
title: ROS 2 Topics and Message Passing
sidebar_position: 5
---

# ROS 2 Topics and Message Passing

## Understanding Topics

Topics in ROS 2 are named buses over which nodes exchange messages. The communication is based on a publish-subscribe model where publishers send messages to a topic and subscribers receive messages from a topic. This creates an asynchronous, many-to-many communication pattern that is perfect for continuous data streams.

In humanoid robotics, topics are commonly used for:
- Sensor data streams (camera images, IMU data, joint states)
- Actuator commands (motor positions, velocities, efforts)
- System status updates (battery levels, error states)
- Perception results (detected objects, navigation goals)

## Quality of Service (QoS) in Humanoid Robotics

Quality of Service settings allow you to specify how messages are delivered, which is crucial for humanoid robots where some data is more time-sensitive than others:

### Reliability
- **Reliable**: Every message is guaranteed to be delivered (e.g., critical safety commands)
- **Best Effort**: Messages may be lost (e.g., video streams where some frames can be dropped)

### Durability
- **Transient Local**: Late-joining subscribers receive the last message (e.g., current robot state)
- **Volatile**: Only new messages are sent (e.g., sensor streams)

### History
- **Keep Last**: Only store the most recent messages
- **Keep All**: Store all messages (use with caution for memory usage)

## Example: Joint State Topic in Humanoid Robots

Here's an example of how topics are used for joint state information in humanoid robots:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher with specific QoS for joint states
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile
        )

        # Timer to publish joint states at regular intervals
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50 Hz

        # Initialize joint names for a simple humanoid
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        self.get_logger().info('Joint State Publisher initialized')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)  # Placeholder values
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        # In a real implementation, these would come from actual sensor readings
        # or controller outputs

        self.joint_state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example: Subscriber for Humanoid Control Commands

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ControlCommandSubscriber(Node):
    def __init__(self):
        super().__init__('control_command_subscriber')

        # Subscribe to velocity commands
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Subscribe to high-level commands
        self.command_subscriber = self.create_subscription(
            String,
            'high_level_commands',
            self.command_callback,
            10
        )

        self.get_logger().info('Control Command Subscriber initialized')

    def cmd_vel_callback(self, msg):
        # Process velocity commands for base movement
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        self.get_logger().info(f'Received velocity command: {linear_x}, {linear_y}, {angular_z}')

        # Forward to base controller
        self.execute_base_movement(linear_x, linear_y, angular_z)

    def command_callback(self, msg):
        # Process high-level commands like "walk", "stand", "sit"
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Execute appropriate behavior
        self.execute_behavior(command)

    def execute_base_movement(self, linear_x, linear_y, angular_z):
        # Implementation would send commands to base controller
        pass

    def execute_behavior(self, command):
        # Implementation would execute the requested behavior
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ControlCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Topic Usage in Humanoid Robotics

1. **Appropriate QoS Settings**: Choose QoS settings based on the criticality of the data
2. **Message Rate**: Balance between responsiveness and computational load
3. **Topic Names**: Use consistent, descriptive naming conventions
4. **Message Types**: Use standard message types when possible for interoperability
5. **Data Bandwidth**: Consider the bandwidth requirements for wireless communication
6. **Synchronization**: Use message filters when you need to synchronize data from multiple topics