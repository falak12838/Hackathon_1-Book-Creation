# Collision Detection and Response

## Overview

This tutorial covers collision detection and response in Gazebo, focusing on how humanoid robots interact with objects in their environment. Understanding collision detection is essential for creating realistic simulation environments.

## Prerequisites

- Basic humanoid model created
- Understanding of physics principles
- Gravitational force application knowledge

## Setting Up Collision Scenarios

### Step 1: Create a World with Obstacles

Create a new world file called `collision_test.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="collision_test_world">
    <!-- Physics Engine Configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Obstacles -->
    <!-- Cube obstacle -->
    <model name="obstacle_cube">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <pose>0 0 0 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Sphere obstacle -->
    <model name="obstacle_sphere">
      <pose>-2 1 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Wall obstacle -->
    <model name="obstacle_wall">
      <pose>0 -3 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Step 2: Launch the World

To launch Gazebo with the collision test world:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Launch Gazebo with the collision test world
gz sim -r collision_test.sdf
```

## Types of Collisions

### 1. Static Collision

This occurs when a moving object collides with a stationary object. In our example, this would be your humanoid robot colliding with the static obstacles.

### 2. Dynamic Collision

This occurs when two moving objects collide. To test this, you could add a moving object to your world:

```xml
<!-- Moving sphere -->
<model name="moving_sphere">
  <pose>-4 0 1 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.3</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.3</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>1 1 0 1</ambient>
        <diffuse>1 1 0 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>0.5</mass>
      <inertia>
        <ixx>1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>1</iyy>
        <iyz>0</iyz>
        <izz>1</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

### 3. Self-Collision

This occurs when parts of the same robot collide. For humanoid robots, self-collision is typically disabled to avoid issues with limbs bumping into each other during normal movement.

## Collision Properties and Parameters

### Contact Properties

You can define specific contact properties for different materials:

```xml
<gazebo reference="link_name">
  <mu1>0.5</mu1>  <!-- Primary friction coefficient -->
  <mu2>0.5</mu2>  <!-- Secondary friction coefficient -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>  <!-- Contact damping -->
  <max_vel>100.0</max_vel>  <!-- Maximum contact penetration error reduction velocity -->
  <min_depth>0.001</min_depth>  <!-- Minimum contact depth -->
</gazebo>
```

### Material Properties

Different materials have different collision behaviors:

- **Rubber**: High friction, high restitution (bounciness)
- **Metal**: Low friction, low restitution
- **Wood**: Medium friction, low restitution

## Collision Detection Algorithms

Gazebo uses the Ignition Physics engine with several collision detection algorithms:

### Bullet Physics
- Fast and robust for most applications
- Good for rigid body simulation
- Recommended for humanoid robots

### ODE (Open Dynamics Engine)
- Stable for complex multi-body systems
- Good for articulated robots
- Default for many ROS 2 applications

### Simbody
- Accurate for complex systems
- Good for biological simulations
- More computationally intensive

## Implementing Collision Detection in Your Model

### Adding Collision Elements to URDF

Make sure your URDF has proper collision elements:

```xml
<link name="base_link">
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </collision>
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <material name="blue">
      <color rgba="0 0 1 0.8"/>
    </material>
  </visual>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

### Collision Checking in Code

You can also check for collisions programmatically using ROS 2:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        self.subscription = self.create_subscription(
            ContactsState,
            '/gazebo/collisions',
            self.collision_callback,
            10)
        self.subscription  # prevent unused variable warning

    def collision_callback(self, msg):
        if len(msg.states) > 0:
            self.get_logger().info(f'Collision detected between: {msg.states[0].collision1_name} and {msg.states[0].collision2_name}')

def main(args=None):
    rclpy.init(args=args)
    collision_detector = CollisionDetector()
    rclpy.spin(collision_detector)
    collision_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Collision Response Analysis

### Response Parameters

When a collision occurs, Gazebo calculates the response based on:

1. **Impulse**: The change in momentum during collision
2. **Restitution**: The "bounciness" of the collision
3. **Friction**: The resistance to sliding motion
4. **Contact area**: The area of contact between objects

### Analyzing Collision Data

You can analyze collision data to understand robot behavior:

- **Force magnitude**: How strong was the collision?
- **Contact points**: Where did the collision occur?
- **Duration**: How long did the contact last?

## Exercise: Collision Testing

1. Create different collision scenarios with your humanoid model:
   - Collision with a wall
   - Collision with a sphere
   - Collision from different angles

2. Modify the friction coefficients and observe the changes in collision behavior

3. Add a sensor to detect collisions and log the data

4. Test with different humanoid poses to see how the collision response varies

## Advanced Collision Concepts

### Soft Contact Modeling

For more realistic interactions, you can use soft contact modeling:

```xml
<physics type="ode">
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.000001</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Collision Avoidance

In real robotics, collision avoidance is critical. You can implement this in simulation using:

- Distance sensors to detect obstacles
- Path planning algorithms
- Reactive collision avoidance behaviors

## Troubleshooting Common Issues

### Issue: Objects pass through each other

**Solution**: Check collision geometry and increase physics update rate:
```xml
<physics type="ode">
  <max_step_size>0.0005</max_step_size>  <!-- Smaller time step -->
  <real_time_update_rate>2000.0</real_time_update_rate>  <!-- Higher update rate -->
</physics>
```

### Issue: Excessive jittering at contact points

**Solution**: Adjust contact parameters:
```xml
<max_vel>1.0</max_vel>  <!-- Lower maximum velocity -->
<min_depth>0.01</min_depth>  <!-- Increase minimum depth -->
```

## Next Steps

After mastering collision detection and response, you have completed the foundational physics simulation concepts. You can now move on to [Chapter 2: Digital Twins & HRI using Unity](../chapter-2-digital-twins-unity/index.md) to learn about visualization and interaction with your simulated humanoid robot.