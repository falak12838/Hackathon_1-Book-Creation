# VLA Integration Setup Guide

This guide covers setting up the complete Vision-Language-Action (VLA) system integration with ROS 2 and humanoid robot control.

## Overview

The complete VLA system integrates voice processing, cognitive planning, and robot execution into a unified autonomous system. This guide will help you set up all components to work together.

## Prerequisites

Before setting up VLA integration, ensure you have:

1. ROS 2 Humble Hawksbill installed
2. Python 3.8 or higher
3. OpenAI API access for Whisper and LLMs
4. All VLA core modules installed

## System Architecture

The VLA system consists of these main components:

```
Voice Command → [Voice Processing] → [Command Interpretation] → [Cognitive Planning] → [Action Execution]
                   ↓                    ↓                        ↓                       ↓
              Whisper API        NLP Analysis            LLM Planning           Robot Control
```

## Installation

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Set Up Environment

Create a comprehensive `.env` file:

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here
WHISPER_MODEL=whisper-1
LLM_MODEL=gpt-3.5-turbo

# ROS 2 Configuration
ROS_DOMAIN_ID=42

# System Configuration
VLA_SYSTEM_NAME=vla_main_system
```

## ROS 2 Node Setup

### 1. Voice Command Node

The voice command node processes audio input and converts to structured commands:

```bash
# Launch the voice command node
ros2 run vla voice_command_node.py
```

### 2. Cognitive Planner Node

The cognitive planner translates high-level commands into action sequences:

```bash
# Launch the cognitive planner node
ros2 run vla cognitive_planner_node.py
```

### 3. Humanoid Controller Node

The controller executes action sequences on the robot:

```bash
# Launch the humanoid controller node
ros2 run vla humanoid_controller_node.py
```

## Configuration Files

### System Configuration

Create a configuration file for your VLA system:

```yaml
# config/vla_system.yaml
vla_system:
  master_node:
    qos_override:
      state_publisher:
        depth: 10
        durability: volatile
      command_publisher:
        depth: 10
        durability: volatile
    planning_timeout: 30.0
    execution_timeout: 300.0  # 5 minutes

  safety_supervisor:
    safety_distance: 0.5  # meters
    emergency_stop_timeout: 10.0  # seconds
    velocity_limits:
      linear_max: 1.0
      angular_max: 1.0

  voice_processing:
    whisper_model: "whisper-1"
    confidence_threshold: 0.7
    audio_buffer_size: 1024

  cognitive_planner:
    llm_model: "gpt-3.5-turbo"
    planning_timeout: 15.0
    max_retries: 3

  performance_monitor:
    cpu_threshold: 80.0
    memory_threshold: 80.0
    disk_threshold: 80.0
```

## Launch Configuration

### Single Launch File

Create a launch file to bring up the entire VLA system:

```python
# launch/vla_system_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # VLA Master Node (main orchestrator)
        Node(
            package='vla',
            executable='vla_master_node',
            name='vla_master_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Safety Supervisor
        Node(
            package='vla',
            executable='safety_supervisor',
            name='safety_supervisor',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Voice Command Node
        Node(
            package='vla',
            executable='voice_command_node',
            name='voice_command_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Cognitive Planner Node
        Node(
            package='vla',
            executable='cognitive_planner_node',
            name='cognitive_planner_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Humanoid Controller Node
        Node(
            package='vla',
            executable='humanoid_controller_node',
            name='humanoid_controller_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        )
    ])
```

### Launch the Complete System

```bash
# Launch the entire VLA system
ros2 launch vla vla_system_launch.py
```

## Testing the Integration

### 1. Basic Functionality Test

Test the complete pipeline with simple commands:

```bash
# Send a simple command to test the pipeline
ros2 topic pub /audio_input std_msgs/String "data: 'Move forward 1 meter'"
```

### 2. Complex Task Test

Test with more complex commands:

```bash
# Test a multi-step command
ros2 topic pub /audio_input std_msgs/String "data: 'Go to the kitchen and bring me the red cup'"
```

## Safety Configuration

### Emergency Stop Setup

Configure the safety supervisor for emergency stops:

```python
# In your safety supervisor node
safety_distance = 0.5  # meters
emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
```

### Safety Monitoring

Enable safety monitoring:

```bash
# Check safety status
ros2 topic echo /safety_status

# Trigger emergency stop if needed
ros2 topic pub /emergency_stop std_msgs/Bool "data: true"
```

## Performance Monitoring

### System Performance

Monitor system performance with the performance monitor:

```bash
# Launch performance monitor
ros2 run vla performance_monitor.py
```

### Resource Usage

Check resource usage during operation:

```bash
# Monitor CPU and memory
ros2 topic echo /system_performance
```

## Troubleshooting

### Common Issues

1. **Node Communication**: Ensure all nodes are on the same ROS_DOMAIN_ID
2. **API Access**: Verify OpenAI API keys and permissions
3. **Audio Input**: Check audio device configuration and permissions
4. **Robot Control**: Verify robot driver connections

### Debugging Tips

- Use `ros2 topic list` to verify all topics are available
- Use `ros2 node list` to check if all nodes are running
- Monitor logs with `ros2 topic echo` for each system component
- Check system resources with performance monitoring tools

## Next Steps

After successful integration setup, explore the complete autonomous capabilities in the capstone section to see how all components work together for complex task execution.