---
title: Robot Structure with URDF
sidebar_position: 3
---

# Robot Structure with URDF

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, and other components. URDF is essential for robot simulation, visualization, and control in ROS-based systems.

## URDF Fundamentals

A URDF file describes a robot as a collection of links connected by joints. This tree-like structure represents the kinematic chain of the robot.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define the physical parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links together -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.25 -0.1" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links in URDF

Links represent the rigid parts of the robot. Each link can have multiple properties:

### Visual Properties
- Define how the link appears in visualization tools
- Include geometry, material, and origin information

### Collision Properties
- Define the collision boundaries for physics simulation
- May be different from visual geometry for performance reasons

### Inertial Properties
- Define the mass and inertial characteristics
- Important for physics simulation and control

## Joints in URDF

Joints connect links and define how they can move relative to each other. Common joint types include:

- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Continuous**: Continuous rotational movement (like a wheel)
- **Prismatic**: Linear sliding movement
- **Planar**: Movement in a plane
- **Floating**: Free movement in 3D space

### Joint Definition Example

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="upper_arm_link"/>
  <origin xyz="0.0 0.2 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## URDF for Humanoid Robots

Humanoid robots have specific structural requirements that need to be accurately represented in URDF:

### Key Components
- **Torso**: The main body of the robot
- **Head**: Contains sensors like cameras
- **Arms**: With shoulders, elbows, wrists, and hands
- **Legs**: With hips, knees, ankles, and feet
- **Joints**: Allowing for human-like movement patterns

### Humanoid-Specific Considerations
- Balance and center of mass are critical
- Multiple degrees of freedom for natural movement
- Proper joint limits to prevent damage
- Accurate inertial properties for stable control

## Creating a Simple Humanoid URDF

Here's an example of a simplified humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Additional joints and links for complete humanoid would continue here -->
</robot>
```

## URDF Validation

Validating your URDF files is crucial to ensure they work correctly in simulation and with control systems.

### Validation Tools

1. **check_urdf**: Command-line tool to validate URDF syntax
   ```bash
   check_urdf /path/to/your/robot.urdf
   ```

2. **URDF parsers**: Most ROS tools will validate URDF on load

### Common Issues to Check
- Proper XML syntax
- All referenced links exist
- Joint parent/child relationships are valid
- Inertial properties are properly defined
- Joint limits are reasonable

## Simulation Readiness

For URDF models to work well in simulation environments like Gazebo:

1. **Proper Inertial Properties**: Accurate mass and inertia values
2. **Collision Meshes**: Appropriate complexity for physics simulation
3. **Joint Dynamics**: Proper friction and damping values
4. **Transmission Definitions**: For actuator control (if needed)

### Gazebo-Specific Elements

To make your URDF compatible with Gazebo, you can add Gazebo-specific tags:

```xml
<link name="wheel_link">
  <visual>
    <geometry>
      <cylinder length="0.1" radius="0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.1" radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>

  <!-- Gazebo-specific properties -->
  <gazebo reference="wheel_link">
    <mu1>0.5</mu1>
    <mu2>0.5</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Blue</material>
  </gazebo>
</link>

<!-- Gazebo plugin for differential drive -->
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```

This example shows how to add Gazebo-specific physics properties and a differential drive controller plugin.

## Best Practices

- Start simple and add complexity gradually
- Use consistent naming conventions
- Validate frequently during development
- Include comments for complex sections
- Test in simulation early and often
- Consider using Xacro for complex models (parametrized URDF)

## Exercises

1. Create a simple URDF model of a wheeled robot
2. Add proper inertial properties to a basic link
3. Define a revolute joint with appropriate limits
## Summary

This chapter covered the fundamentals of URDF (Unified Robot Description Format) for representing robot structure in ROS. You learned about links, joints, visual and collision properties, and how to make your models simulation-ready with Gazebo integration.

## Next Steps

You've completed Module 1: The Robotic Nervous System. This covers the essential concepts of ROS 2 as middleware for humanoid robots, including communication patterns and robot structure representation.

[Previous: ROS 2 Communication Model](./ros2-communication-model.md)