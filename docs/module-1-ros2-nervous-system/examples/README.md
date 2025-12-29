# ROS 2 Communication Examples

This directory contains example code for the ROS 2 communication model chapter.

## Available Examples

1. `humanoid_agent.py` - An agent node that processes high-level commands
2. `humanoid_controller.py` - A controller node that executes low-level joint commands

## Setup Instructions

1. Make sure your ROS 2 environment is sourced:
```bash
source /opt/ros/humble/setup.bash
```

2. Create a workspace for the examples:
```bash
mkdir -p ~/ros2_examples/src
cd ~/ros2_examples
```

3. Copy the example files to your workspace:
```bash
cp /path/to/humanoid_agent.py src/
cp /path/to/humanoid_controller.py src/
```

4. Build the workspace:
```bash
colcon build
source install/setup.bash
```

## Running the Examples

To run the agent and controller nodes:

1. Terminal 1 - Run the controller:
```bash
ros2 run your_package humanoid_controller.py
```

2. Terminal 2 - Run the agent:
```bash
ros2 run your_package humanoid_agent.py
```

3. Terminal 3 - Send commands to the agent:
```bash
ros2 topic pub /robot_commands std_msgs/String "data: 'move_forward'"
```

## Dependencies

These examples require the following ROS 2 packages:
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `trajectory_msgs`