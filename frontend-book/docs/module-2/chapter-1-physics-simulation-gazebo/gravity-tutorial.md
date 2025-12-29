# Gravitational Force Application

## Overview

This tutorial demonstrates how to apply gravitational forces to your humanoid robot model in Gazebo and observe realistic falling motion. Understanding gravity application is fundamental to physics simulation in robotics.

## Prerequisites

- Basic humanoid model created (completed in Basic Humanoid Model Creation)
- Understanding of physics principles (completed in Physics Principles section)
- Gazebo Harmonic installed and configured

## Setting Up the Environment

### Step 1: Create a World File with Different Gravity Settings

Create a new world file called `gravity_test.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="gravity_test_world">
    <!-- Physics Engine Configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>  <!-- Standard Earth gravity -->
    </physics>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Your Humanoid Model -->
    <!-- This will be spawned separately using ROS 2 tools -->
  </world>
</sdf>
```

### Step 2: Launch Gazebo with Custom World

To launch Gazebo with your custom world:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Launch Gazebo with the custom world
gz sim -r gravity_test.sdf
```

## Observing Gravitational Effects

### Scenario 1: Standard Gravity (9.8 m/s²)

1. Position your humanoid model above the ground (e.g., at z=2.0)
2. Observe how gravity causes the model to fall
3. Note the acceleration and impact with the ground

In your URDF, make sure your model starts at an elevated position:

```xml
<spawn>
  <name>simple_humanoid</name>
  <pose>0 0 2 0 0 0</pose>  <!-- Start 2 meters above ground -->
</spawn>
```

### Scenario 2: Zero Gravity

Create a world file with zero gravity to observe the difference:

```xml
<world name="zero_gravity">
  <physics type="ode">
    <gravity>0 0 0</gravity>  <!-- No gravity -->
  </physics>
  <!-- Other elements -->
</world>
```

### Scenario 3: Different Gravity Values

Experiment with different gravity values to understand their effects:

- **Moon gravity** (1.62 m/s²): `<gravity>0 0 -1.62</gravity>`
- **Mars gravity** (3.71 m/s²): `<gravity>0 0 -3.71</gravity>`
- **Jupiter gravity** (24.79 m/s²): `<gravity>0 0 -24.79</gravity>`

## Creating a Gravity Test Launch File

Create a launch file to easily test different gravity scenarios:

Create `gravity_test.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch argument for gravity value
    gravity_arg = DeclareLaunchArgument(
        'gravity_z',
        default_value='-9.8',
        description='Gravity value in Z direction'
    )

    gravity_z = LaunchConfiguration('gravity_z')

    return LaunchDescription([
        gravity_arg,

        # Create a temporary world file with specified gravity
        ExecuteProcess(
            cmd=['bash', '-c',
                f'echo \'<?xml version="1.0"?><sdf version="1.7"><world name="gravity_test"><physics type="ode"><gravity>0 0 {gravity_z}</gravity></physics><include><uri>model://ground_plane</uri></include><include><uri>model://sun</uri></include></world></sdf>\' > /tmp/test_gravity.sdf'],
            output='screen'
        ),

        # Launch Gazebo with the world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '/tmp/test_gravity.sdf'],
            output='screen'
        ),

        # Spawn the humanoid robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_humanoid',
                      '-file', os.path.expanduser('~/simple_humanoid.urdf'),
                      '-x', '0', '-y', '0', '-z', '2.0'],
            output='screen'
        ),
    ])
```

## Understanding the Physics

### Free Fall Motion

When an object is in free fall under gravity, its motion follows the kinematic equation:

`h(t) = h₀ + v₀t + ½gt²`

Where:
- `h(t)` is the height at time t
- `h₀` is the initial height
- `v₀` is the initial velocity
- `g` is the gravitational acceleration
- `t` is time

### Impact and Collision Response

When your humanoid model impacts the ground, Gazebo calculates the collision response based on:
- Mass of the colliding objects
- Velocity at impact
- Material properties (friction, restitution)
- Contact geometry

## Measuring and Analyzing Results

### Using Gazebo Tools

You can use Gazebo's built-in tools to analyze motion:

1. **Plotting**: Use the plot tools to visualize position, velocity, and acceleration over time
2. **Sensors**: Add IMU sensors to measure acceleration directly
3. **Logging**: Log position data to analyze the motion mathematically

### Adding an IMU Sensor

To measure acceleration, add an IMU sensor to your humanoid model:

```xml
<!-- Add to one of your links, e.g., base_link -->
<gazebo>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
    <topic>imu/data</topic>
    <update_rate>100</update_rate>
  </plugin>
</gazebo>

<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <visualize>true</visualize>
</sensor>
```

## Exercise: Gravity Analysis

1. Create a simple script to measure the time it takes for your humanoid to fall from different heights
2. Compare the measured times with the theoretical values: `t = √(2h/g)`
3. Test with different gravity values and record the differences
4. Add a sphere of the same mass as your humanoid and compare their falling times (they should be identical in vacuum)

## Advanced Gravity Concepts

### Gravity in Multi-Body Systems

For complex humanoid robots with multiple links:
- Each link experiences gravity individually
- Joint constraints affect how gravity influences the overall system
- Center of mass determines the effective gravitational force point

### Non-Standard Gravity Scenarios

- **Rotating reference frames**: Simulate gravity effects in rotating space stations
- **Variable gravity**: Simulate gradual changes in gravitational field
- **Microgravity**: Simulate space environments

## Next Steps

After understanding gravitational force application, proceed to learn about [Collision Detection and Response](./collision-tutorial.md) to complete your understanding of basic physics simulation.