# Basic Humanoid Model Creation

## Overview

In this tutorial, you will learn how to create a basic humanoid model in Gazebo. This model will serve as the foundation for all subsequent physics simulations in this module.

## Prerequisites

- Gazebo Harmonic installed and configured (completed in the Setup Guide)
- Basic understanding of URDF (Unified Robot Description Format)
- ROS 2 Humble Hawksbill environment sourced

## Creating a Simple Humanoid Model

### Step 1: Create the URDF File

Create a new file called `simple_humanoid.urdf` in your workspace:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_leg">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_leg_joint" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="-0.1 0 -0.55"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_leg">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_leg_joint" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0.1 0 -0.55"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_arm_joint" type="fixed">
    <parent link="base_link"/>
    <child link="left_arm"/>
    <origin xyz="0 -0.15 -0.1" rpy="1.57 0 0"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_arm_joint" type="fixed">
    <parent link="base_link"/>
    <child link="right_arm"/>
    <origin xyz="0 0.15 -0.1" rpy="-1.57 0 0"/>
  </joint>
</robot>
```

### Step 2: Load the Model in Gazebo

To load your model in Gazebo, first make sure the ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

Then use the following command to launch Gazebo with your model:

```bash
# Create a temporary SDF from your URDF
gz sdf -p simple_humanoid.urdf > simple_humanoid.sdf

# Launch Gazebo and insert the model
gz sim -r simple_humanoid.sdf
```

Alternatively, you can create a launch file for easier loading:

Create a file called `humanoid.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file = os.path.join(os.path.expanduser('~'), 'simple_humanoid.urdf')

    return LaunchDescription([
        # Launch Gazebo with empty world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_humanoid', '-file', urdf_file],
            output='screen'
        ),
    ])
```

### Step 3: Verify Model in Gazebo

Once loaded, you should see your simple humanoid model in the Gazebo environment. You can:
- Rotate the camera to view the model from different angles
- Check that all links are properly connected
- Verify that the collision and visual properties are correct

## Understanding the Model Structure

The simple humanoid model consists of:
- A base body (torso)
- A head
- Two arms
- Two legs

Each component is connected via joints. In this example, we used fixed joints, which means the limbs don't move relative to the body. In more advanced models, you would use revolute joints to allow for movement.

## Physics Properties

The model includes important physics properties:
- **Mass**: Each link has a realistic mass value
- **Inertia**: Proper inertia tensors for realistic physics simulation
- **Collision**: Collision properties for interaction with the environment

## Exercise

1. Modify the URDF file to change the colors of different body parts
2. Adjust the dimensions of the links to create a taller or shorter humanoid
3. Add additional elements like fingers or a neck joint

## Next Steps

After creating your basic humanoid model, proceed to learn about [Physics Principles in Simulation](./physics-principles.md) to understand how physical laws apply to your model.