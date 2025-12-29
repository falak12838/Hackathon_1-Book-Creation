# Humanoid Robot Models in Isaac Sim

## Overview

Humanoid robot models in Isaac Sim require careful configuration to ensure realistic simulation and proper integration with ROS 2. These models must accurately represent the physical robot's kinematics, dynamics, and sensor placement while maintaining computational efficiency for real-time simulation.

## URDF (Unified Robot Description Format)

### URDF Basics for Humanoid Robots

URDF is the standard format for describing robot models in ROS and Isaac Sim. For humanoid robots, the structure is more complex than simple wheeled robots due to multiple limbs and joints.

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base/Root Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso Link -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Joint connecting base to torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15"/>
  </joint>
</robot>
```

### Humanoid-Specific URDF Considerations

Humanoid robots have complex kinematic chains:

```xml
<!-- Example of leg structure -->
<link name="left_hip">
  <visual>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
  </inertial>
</link>

<joint name="left_hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_hip"/>
  <origin xyz="0 -0.1 -0.25"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Isaac Sim Robot Import

### Importing URDF to Isaac Sim

Isaac Sim provides tools to import URDF models:

```python
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World

# Import humanoid robot from URDF
def import_humanoid_robot(urdf_path, prim_path="/World/Humanoid"):
    """Import a humanoid robot from URDF file"""
    # Add the URDF to the current stage
    add_reference_to_stage(
        usd_path=urdf_path,
        prim_path=prim_path
    )

    return prim_path

# Example usage
world = World(stage_units_in_meters=1.0)
humanoid_path = import_humanoid_robot(
    urdf_path="path/to/humanoid.urdf",
    prim_path="/World/HumanoidRobot"
)
```

### Articulation and Joint Configuration

Humanoid robots require complex articulation:

```python
from omni.isaac.core.articulations import ArticulationView

# Configure the humanoid as an articulation
humanoid = world.scene.add(
    ArticulationView(
        prim_path="/World/HumanoidRobot",
        name="humanoid_robot"
    )
)

# Access joint information
joint_names = humanoid.dof_names
joint_positions = humanoid.get_joint_positions()
joint_velocities = humanoid.get_joint_velocities()
```

## Humanoid Robot Model Requirements

### Joint Definitions

Humanoid robots typically have these joint types:

```xml
<!-- Hip joints (3 DOF each leg) -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_hip"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="2.0"/>
</joint>

<joint name="left_hip_roll" type="revolute">
  <parent link="left_hip"/>
  <child link="left_thigh"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.3" upper="1.0" effort="100" velocity="2.0"/>
</joint>

<joint name="left_hip_pitch" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <axis xyz="1 0 0"/>
  <limit lower="-2.0" upper="0.5" effort="100" velocity="2.0"/>
</joint>
```

### Link Properties

Each link must have proper physical properties:

```xml
<link name="left_foot">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0015"/>
  </inertial>
</link>
```

## Sensor Integration

### Mounting Sensors on Humanoid Robots

Proper sensor placement is crucial for humanoid robots:

```python
# Example of mounting sensors on a humanoid robot
def mount_sensors_on_humanoid(robot_prim_path):
    """Mount various sensors on the humanoid robot"""

    # Mount camera on head
    camera = Camera(
        prim_path=f"{robot_prim_path}/head_camera",
        frequency=30,
        resolution=(640, 480)
    )
    camera.set_world_pose(
        position=np.array([0.0, 0.0, 0.1]),  # Offset from head link
        orientation=np.array([0.707, 0.0, 0.0, 0.707])  # Looking forward
    )

    # Mount IMU on torso
    imu = ImuSensor(
        prim_path=f"{robot_prim_path}/torso_imu",
        position=np.array([0.0, 0.0, 0.0]),
        frequency=100
    )

    # Mount LiDAR on head
    lidar = create_lidar_sensor(
        prim_path=f"{robot_prim_path}/head_lidar",
        position=np.array([0.05, 0.0, 0.1])
    )

    return camera, imu, lidar
```

### Sensor Configuration for Humanoid Perception

```python
# Configuration for humanoid-specific sensors
humanoid_sensor_config = {
    "head_camera": {
        "resolution": [1280, 720],
        "fov": 90.0,  # Wide field of view for navigation
        "mount_point": "head_link",
        "offset": [0.05, 0.0, 0.05]  # Position relative to head
    },
    "torso_imu": {
        "frequency": 100,
        "mount_point": "torso_link",
        "offset": [0.0, 0.0, 0.0]
    },
    "head_lidar": {
        "max_range": 20.0,
        "samples": 1080,
        "mount_point": "head_link",
        "offset": [0.1, 0.0, 0.05]
    }
}
```

## Balance and Locomotion Considerations

### Center of Mass and Stability

Humanoid robots require careful attention to center of mass:

```python
def calculate_humanoid_stability(robot_mass_properties):
    """Calculate stability metrics for humanoid robot"""
    # Calculate center of mass
    total_mass = sum(link['mass'] for link in robot_mass_properties)
    com_x = sum(link['mass'] * link['com'][0] for link in robot_mass_properties) / total_mass
    com_y = sum(link['mass'] * link['com'][1] for link in robot_mass_properties) / total_mass
    com_z = sum(link['mass'] * link['com'][2] for link in robot_mass_properties) / total_mass

    # Calculate support polygon (simplified)
    support_points = get_foot_positions(robot_mass_properties)
    support_polygon = calculate_support_polygon(support_points)

    # Check if COM is within support polygon
    is_stable = point_in_polygon([com_x, com_y], support_polygon)

    return {
        'center_of_mass': [com_x, com_y, com_z],
        'is_stable': is_stable,
        'support_polygon': support_polygon
    }
```

### Joint Limits and Safety Constraints

```xml
<!-- Example of safety-constrained joints -->
<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <axis xyz="1 0 0"/>
  <limit
    lower="0.1"     <!-- Minimum bend angle -->
    upper="2.5"     <!-- Maximum bend angle -->
    effort="150"    <!-- Maximum torque -->
    velocity="5.0"/> <!-- Maximum velocity -->
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

## Isaac Sim Extensions for Humanoid Robots

### Custom Controllers

Isaac Sim supports custom controllers for humanoid locomotion:

```python
from omni.isaac.core.controllers import BaseController
import numpy as np

class HumanoidBalanceController(BaseController):
    def __init__(self, name="humanoid_balance_controller"):
        super().__init__(name=name)
        self._dof_names = []
        self._num_dof = 0

    def forward(self,
                current_joint_positions,
                current_joint_velocities,
                target_positions=None,
                target_velocities=None):
        """Calculate control commands for humanoid balance"""

        # Implement balance control logic
        # This is a simplified example
        if target_positions is None:
            target_positions = current_joint_positions

        # Calculate position errors
        position_errors = target_positions - current_joint_positions

        # Simple PD control
        kp = 100.0  # Proportional gain
        kd = 10.0   # Derivative gain

        control_commands = kp * position_errors - kd * current_joint_velocities

        return control_commands
```

### Physics Simulation Parameters

```python
# Physics parameters optimized for humanoid simulation
humanoid_physics_params = {
    "solver_type": "TGS",  # Time-stepping Gauss-Seidel
    "bounce_threshold": 0.1,  # Velocity threshold for bouncing
    "friction_offset_threshold": 0.001,  # Friction offset threshold
    "ccd_max_passes": 1,  # Continuous collision detection passes
    "max_depenetration_velocity": 100.0  # Maximum penetration recovery velocity
}
```

## Validation and Testing

### Model Validation

Validate the humanoid robot model before simulation:

```python
def validate_humanoid_model(urdf_path):
    """Validate humanoid robot model for Isaac Sim compatibility"""
    import xml.etree.ElementTree as ET

    # Parse URDF
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    issues = []

    # Check for required links
    links = [link.get('name') for link in root.findall('link')]
    required_links = ['base_link', 'torso', 'head']
    missing_links = [link for link in required_links if link not in links]
    if missing_links:
        issues.append(f"Missing required links: {missing_links}")

    # Check joint types
    joints = root.findall('joint')
    for joint in joints:
        joint_type = joint.get('type')
        if joint_type not in ['revolute', 'prismatic', 'fixed', 'continuous']:
            issues.append(f"Invalid joint type '{joint_type}' for joint {joint.get('name')}")

    # Check for proper mass properties
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            issues.append(f"Link {link.get('name')} missing inertial properties")
        else:
            mass = inertial.find('mass')
            if mass is None or float(mass.get('value')) <= 0:
                issues.append(f"Link {link.get('name')} has invalid mass")

    return issues
```

### Simulation Testing

```python
def test_humanoid_simulation(world, humanoid_robot):
    """Test humanoid robot simulation stability"""

    # Reset the world
    world.reset()

    # Run simulation for a short period
    initial_positions = humanoid_robot.get_joint_positions()

    for i in range(100):  # Run for 100 steps
        world.step(render=True)

        # Check for stability
        current_positions = humanoid_robot.get_joint_positions()

        # Check for joint limits
        joint_limits_violated = check_joint_limits(
            current_positions,
            humanoid_robot.get_dof_limits()
        )

        if joint_limits_violated:
            return False, f"Joint limits violated at step {i}"

    # Check if robot remained stable
    final_positions = humanoid_robot.get_joint_positions()
    position_change = np.linalg.norm(final_positions - initial_positions)

    if position_change > 1.0:  # If robot moved too much without control
        return False, "Robot unstable without control input"

    return True, "Model passed stability test"
```

## Best Practices

### Model Design Guidelines

1. **Realistic Mass Distribution**: Ensure proper mass and inertia properties
2. **Appropriate Joint Limits**: Set realistic joint limits based on physical constraints
3. **Collision Geometry**: Use simplified collision geometry for performance
4. **Visual vs. Collision**: Separate visual and collision geometry appropriately

### Performance Optimization

- **Simplified Collision Models**: Use boxes and cylinders instead of complex meshes
- **LOD Systems**: Implement level of detail for complex models
- **Joint Damping**: Add appropriate damping to prevent oscillations
- **Mass Distribution**: Optimize center of mass for stability

### Documentation

- **URDF Comments**: Document joint purposes and limitations
- **Configuration Files**: Maintain separate config files for different robot variants
- **Validation Scripts**: Create scripts to validate model changes
- **Testing Procedures**: Document testing procedures for new models

## Next Steps

With humanoid robot models understood, you're ready to explore:

- Advanced locomotion and balance control in Isaac Sim
- Integration with Isaac ROS for perception and control
- Navigation planning for humanoid robots
- Sensor fusion for humanoid robot perception

Continue to the next section to learn about validation techniques for Isaac Sim environments.