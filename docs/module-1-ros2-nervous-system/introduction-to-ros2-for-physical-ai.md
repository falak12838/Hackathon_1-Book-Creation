---
title: Introduction to ROS 2 for Physical AI
sidebar_position: 1
---

# Introduction to ROS 2 for Physical AI

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike a traditional operating system, ROS 2 is middleware that provides services designed specifically for robotic applications, such as hardware abstraction, device drivers, libraries for implementing common robot functionality, message-passing between processes, and tools for package management and visualization.

## Why ROS 2 Matters for Humanoids

Humanoid robots present unique challenges that make ROS 2 particularly valuable:

1. **Complexity Management**: Humanoids have many degrees of freedom, requiring coordination between multiple subsystems like perception, planning, control, and execution.

2. **Modularity**: ROS 2's node-based architecture allows different teams to work on different aspects of the robot (e.g., vision, locomotion, manipulation) independently.

3. **Scalability**: The distributed nature of ROS 2 allows computation to be spread across multiple processors and computers within the robot.

4. **Community**: A large community of researchers and developers contribute packages and share knowledge, accelerating development.

5. **Real-time Performance**: Modern ROS 2 distributions include real-time capabilities essential for robot control.

## Core Architecture

ROS 2 is built on the Data Distribution Service (DDS) standard, which provides:

- **Publisher/Subscriber Pattern**: Nodes communicate through topics
- **Client/Server Pattern**: Services for request/response interactions
- **Parameters**: Configuration management across nodes
- **Actions**: Goal-oriented communication with feedback

This architecture enables the creation of distributed robotic systems where different components can run on different machines while maintaining seamless communication.

## ROS 2 vs. Traditional Approaches

Compared to building robot software from scratch or using proprietary systems, ROS 2 offers:

- **Standardized Interfaces**: Common message types and services
- **Reusability**: Many existing packages for common robot functions
- **Simulation Integration**: Tight integration with simulation environments like Gazebo
- **Hardware Abstraction**: Consistent interfaces across different hardware platforms
- **Debugging Tools**: Built-in tools for visualization, logging, and debugging

## OOS Concepts (Open, Operate, Share)

The OOS philosophy is central to the ROS 2 ecosystem and collaborative robotics development:

### Open

- **Open Source**: ROS 2 is developed in the open with a large community of contributors
- **Open Standards**: Uses standard protocols like DDS for communication
- **Open Development**: Encourages sharing of packages and solutions across the community
- **Transparent Design**: Architecture decisions are documented and community-driven

### Operate

- **Reliability**: Designed for real-world robotic applications with fault tolerance
- **Deterministic Behavior**: Predictable performance in time-critical applications
- **Safety First**: Built-in mechanisms for safe robot operation
- **Real-time Capabilities**: Support for time-sensitive robotic control

### Share

- **Package Reuse**: Thousands of packages available for common robot functions
- **Community Collaboration**: Shared tools, tutorials, and solutions
- **Standardized Interfaces**: Common APIs across different robots and applications
- **Knowledge Transfer**: Best practices and lessons learned shared across the community

## Getting Started with ROS 2

Before diving into development, it's important to understand the basic concepts that form the foundation of ROS 2:

- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Goal-oriented communication with feedback
- **Parameters**: Configuration values that can be set at runtime

### Simple Node Example

Here's a basic example of a ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates the basic structure of a ROS 2 node, including initialization, publisher creation, and a timer callback for publishing messages.

### ROS 2 Architecture

The ROS 2 architecture consists of nodes that communicate through topics, services, and actions, all managed by the DDS (Data Distribution Service) middleware layer. This enables communication between different components of a robotic system.

## ROS 2 Environment Setup

To work with ROS 2, you'll need to set up your development environment. We recommend using ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) version suitable for educational purposes.

### Installation

For Ubuntu (recommended for humanoid robotics development):

```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update apt and install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
```

For other platforms, refer to the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

### Workspace Setup

Create a workspace for your ROS 2 projects:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Troubleshooting Common Issues

### Environment Not Sourcing Properly

If ROS 2 commands are not recognized, make sure you've sourced the environment:

```bash
source /opt/ros/humble/setup.bash
```

To make this permanent, add it to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Package Installation Failures

If you encounter GPG key errors during installation:

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Workspace Build Issues

If `colcon build` fails, ensure all dependencies are installed:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Exercises

1. Set up your ROS 2 environment following the instructions above
2. Create a simple ROS 2 workspace
3. Verify your installation by running `ros2 topic list`

## Summary

This chapter introduced the fundamental concepts of ROS 2 and its importance for humanoid robotics. You learned about the OOS philosophy and got your development environment set up.

## Next Steps

Continue to the next chapter to learn about the ROS 2 communication model:

[Next: ROS 2 Communication Model](./ros2-communication-model.md)