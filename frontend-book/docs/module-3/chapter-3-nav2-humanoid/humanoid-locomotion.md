# Humanoid Locomotion Constraints in Nav2

## Overview

Humanoid locomotion involves complex biomechanical and control challenges that significantly impact navigation planning and execution. Unlike wheeled robots, humanoid robots must manage balance, coordinate multiple joints, and execute complex gait patterns. This section details the key locomotion constraints that must be considered when implementing Nav2 for humanoid robots.

## Fundamentals of Humanoid Locomotion

### Gait Patterns

Humanoid robots typically use gait patterns that mimic human walking:

#### Walking Gait Phases
- **Double Support Phase**: Both feet in contact with ground
- **Single Support Phase**: One foot in contact with ground
- **Swing Phase**: Foot moving forward through air
- **Contact Phase**: Foot making contact with ground

```python
# Gait phase definitions
class GaitPhases:
    DOUBLE_SUPPORT = "double_support"
    SINGLE_SUPPORT = "single_support"
    SWING = "swing"
    CONTACT = "contact"

class GaitPattern:
    def __init__(self, step_duration=1.0, double_support_ratio=0.2):
        self.step_duration = step_duration  # Total time for one step
        self.double_support_ratio = double_support_ratio  # Ratio of double support time
        self.single_support_duration = step_duration * (1 - double_support_ratio)
        self.double_support_duration = step_duration * double_support_ratio

    def get_current_phase(self, time_in_step):
        """Determine current gait phase based on time in step"""
        if time_in_step < self.double_support_duration / 2:
            return GaitPhases.DOUBLE_SUPPORT  # Initial double support
        elif time_in_step < self.double_support_duration / 2 + self.single_support_duration:
            return GaitPhases.SINGLE_SUPPORT  # Single support
        else:
            return GaitPhases.DOUBLE_SUPPORT  # Final double support
```

#### Steady-State Walking
- **Periodic Pattern**: Repeatable walking pattern
- **Step Length**: Distance between consecutive foot placements
- **Step Width**: Lateral distance between feet
- **Step Timing**: Temporal characteristics of steps

#### Transient Gaits
- **Starting**: Transition from standing to walking
- **Stopping**: Transition from walking to standing
- **Turning**: Lateral movement and rotation
- **Speed Changes**: Acceleration and deceleration

### Balance Control Fundamentals

#### Zero Moment Point (ZMP) Theory
- **ZMP Definition**: Point where the moment of ground reaction force equals zero
- **Support Polygon**: Convex hull of ground contact points
- **ZMP Stability**: ZMP must remain within support polygon
- **Trajectory Planning**: Plan ZMP trajectory for stable walking

```python
# ZMP-based balance control
class ZMPController:
    def __init__(self, robot_config):
        self.robot_config = robot_config
        self.com_height = robot_config['com_height']  # Center of mass height
        self.gravity = 9.81

    def calculate_zmp(self, ground_reaction_force, ground_reaction_moment):
        """Calculate ZMP from ground reaction forces and moments"""
        if abs(ground_reaction_force[2]) < 0.1:  # Avoid division by zero
            return [0.0, 0.0]

        zmp_x = (ground_reaction_moment[1] + ground_reaction_force[0] * self.com_height) / ground_reaction_force[2]
        zmp_y = (-ground_reaction_moment[0] + ground_reaction_force[1] * self.com_height) / ground_reaction_force[2]

        return [zmp_x, zmp_y]

    def is_stable(self, zmp, support_polygon):
        """Check if ZMP is within support polygon"""
        return self.point_in_polygon(zmp, support_polygon)

    def point_in_polygon(self, point, polygon):
        """Check if point is inside polygon using ray casting"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside
```

#### Center of Mass (CoM) Control
- **CoM Position**: Maintain CoM within stable region
- **CoM Velocity**: Control CoM movement speed
- **CoM Acceleration**: Manage CoM acceleration for stability
- **Capture Point**: Predictive control point for balance

## Locomotion Constraints for Navigation

### Kinematic Constraints

#### Joint Limitations
- **Hip Joint Limits**: Range of motion for hip joints
- **Knee Joint Limits**: Flexion/extension constraints
- **Ankle Joint Limits**: Dorsiflexion/plantarflexion limits
- **Arm Coordination**: Arm movements for balance

```python
# Joint limit constraints
class JointConstraints:
    def __init__(self):
        self.joint_limits = {
            'hip_yaw': {'min': -0.5, 'max': 0.5},      # Hip abduction/adduction
            'hip_roll': {'min': -0.3, 'max': 1.0},     # Hip flexion/extension
            'hip_pitch': {'min': -2.0, 'max': 0.5},    # Hip rotation
            'knee_pitch': {'min': 0.0, 'max': 2.5},    # Knee flexion
            'ankle_pitch': {'min': -0.5, 'max': 0.5},  # Ankle dorsiflexion
            'ankle_roll': {'min': -0.2, 'max': 0.2}    # Ankle inversion/eversion
        }

    def validate_joint_angles(self, joint_angles):
        """Validate joint angles against limits"""
        for joint_name, angle in joint_angles.items():
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                if angle < limits['min'] or angle > limits['max']:
                    return False, f"Joint {joint_name} exceeds limits: {angle} not in [{limits['min']}, {limits['max']}]"

        return True, "All joint angles are within limits"
```

#### Step Constraints
- **Maximum Step Length**: Longest step the robot can take
- **Maximum Step Height**: Highest step over obstacles
- **Minimum Step Length**: Shortest stable step
- **Step Width Constraints**: Lateral step limitations

```python
# Step constraint validation
class StepConstraints:
    def __init__(self):
        self.max_step_length = 0.30  # meters
        self.max_step_height = 0.15  # meters
        self.min_step_length = 0.10  # meters
        self.max_step_width = 0.25   # meters (distance between feet)
        self.min_step_time = 0.5     # seconds
        self.max_step_time = 2.0     # seconds

    def validate_step(self, start_pose, end_pose, step_time):
        """Validate a step against locomotion constraints"""
        # Calculate step length
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        dz = end_pose.position.z - start_pose.position.z

        step_length = math.sqrt(dx*dx + dy*dy)
        step_height = abs(dz)

        # Check length constraint
        if step_length > self.max_step_length:
            return False, f"Step length {step_length:.2f}m exceeds maximum {self.max_step_length}m"

        if step_length < self.min_step_length:
            return False, f"Step length {step_length:.2f}m below minimum {self.min_step_length}m"

        # Check height constraint
        if step_height > self.max_step_height:
            return False, f"Step height {step_height:.2f}m exceeds maximum {self.max_step_height}m"

        # Check time constraint
        if step_time < self.min_step_time:
            return False, f"Step time {step_time:.2f}s below minimum {self.min_step_time}s"

        if step_time > self.max_step_time:
            return False, f"Step time {step_time:.2f}s exceeds maximum {self.max_step_time}s"

        return True, "Step is valid for humanoid locomotion"
```

### Dynamic Constraints

#### Balance Requirements
- **Stability Margin**: Safety margin for balance maintenance
- **Angular Momentum**: Control of whole-body angular momentum
- **Impact Forces**: Managing forces during foot contact
- **Energy Efficiency**: Minimizing energy consumption

#### Speed and Acceleration Limits
- **Maximum Walking Speed**: Top speed for stable walking
- **Acceleration Limits**: Safe acceleration/deceleration rates
- **Turning Rate**: Maximum angular velocity
- **Gait Transition Speed**: Speed for changing gaits

```python
# Dynamic constraints for navigation
class DynamicConstraints:
    def __init__(self):
        self.max_linear_speed = 0.4    # m/s
        self.max_angular_speed = 0.5   # rad/s
        self.max_linear_accel = 0.5    # m/s^2
        self.max_angular_accel = 0.5   # rad/s^2
        self.min_step_frequency = 0.5  # Hz
        self.max_step_frequency = 2.0  # Hz

    def validate_velocity_profile(self, linear_vel, angular_vel, linear_acc, angular_acc):
        """Validate velocity and acceleration profiles"""
        if abs(linear_vel) > self.max_linear_speed:
            return False, f"Linear velocity {linear_vel:.2f} exceeds maximum {self.max_linear_speed}"

        if abs(angular_vel) > self.max_angular_speed:
            return False, f"Angular velocity {angular_vel:.2f} exceeds maximum {self.max_angular_speed}"

        if abs(linear_acc) > self.max_linear_accel:
            return False, f"Linear acceleration {linear_acc:.2f} exceeds maximum {self.max_linear_accel}"

        if abs(angular_acc) > self.max_angular_accel:
            return False, f"Angular acceleration {angular_acc:.2f} exceeds maximum {self.max_angular_accel}"

        return True, "Velocity and acceleration profiles are valid"
```

## Integration with Nav2 Path Planning

### Path Planning with Locomotion Constraints

#### Step-Aware Path Generation
- **Discrete Footstep Planning**: Plan discrete foot placements
- **Continuous Path Smoothing**: Smooth paths between footsteps
- **Kinematic Feasibility**: Verify paths are kinematically feasible
- **Dynamic Feasibility**: Ensure paths are dynamically feasible

```python
# Step-aware path planning
class StepAwarePathPlanner:
    def __init__(self, step_constraints, dynamic_constraints):
        self.step_constraints = step_constraints
        self.dynamic_constraints = dynamic_constraints
        self.footstep_planner = FootstepPlanner()

    def plan_path_with_locomotion_constraints(self, start, goal, map):
        """Plan path considering locomotion constraints"""
        # First, plan a basic path
        basic_path = self.plan_basic_path(start, goal, map)

        # Validate and adjust for locomotion constraints
        constrained_path = self.apply_locomotion_constraints(basic_path)

        # Generate footstep plan
        footsteps = self.footstep_planner.plan_footsteps(constrained_path, start)

        return constrained_path, footsteps

    def apply_locomotion_constraints(self, path):
        """Apply locomotion constraints to path"""
        constrained_path = []

        # Ensure path points are within step constraints
        i = 0
        while i < len(path.poses):
            current_point = path.poses[i]
            constrained_path.append(current_point)

            # Find next point that satisfies step constraints
            j = i + 1
            while j < len(path.poses):
                next_point = path.poses[j]

                # Check if step from current to next is valid
                is_valid, _ = self.step_constraints.validate_step(
                    current_point.pose.position,
                    next_point.pose.position,
                    step_time=1.0  # Assume 1 second step time for validation
                )

                if is_valid:
                    # This step is valid, add it to constrained path
                    constrained_path.append(next_point)
                    i = j
                    break
                else:
                    # This step is too large, need intermediate points
                    intermediate = self.create_intermediate_point(current_point, next_point)
                    constrained_path.append(intermediate)
                    # Continue from next point
                    i = j
                    break

                j += 1

            if j >= len(path.poses):
                # Reached end of path
                break

        return constrained_path

    def create_intermediate_point(self, start, end):
        """Create intermediate point between two points"""
        intermediate = PoseStamped()
        intermediate.header = start.header

        # Calculate midpoint
        intermediate.pose.position.x = (start.pose.position.x + end.pose.position.x) / 2.0
        intermediate.pose.position.y = (start.pose.position.y + end.pose.position.y) / 2.0
        intermediate.pose.position.z = (start.pose.position.z + end.pose.position.z) / 2.0

        # Interpolate orientation
        intermediate.pose.orientation = self.interpolate_orientation(
            start.pose.orientation,
            end.pose.orientation
        )

        return intermediate

    def interpolate_orientation(self, start_orientation, end_orientation):
        """Interpolate between two orientations"""
        # For simplicity, return start orientation
        # In practice, use spherical linear interpolation (SLERP)
        return start_orientation
```

### Gait-Aware Trajectory Generation

#### Trajectory Smoothing
- **Gait Pattern Integration**: Integrate gait patterns into trajectories
- **Smooth Transitions**: Ensure smooth transitions between gaits
- **Timing Coordination**: Coordinate trajectory timing with gait
- **Balance Maintenance**: Maintain balance during trajectory execution

```python
# Gait-aware trajectory generation
class GaitAwareTrajectoryGenerator:
    def __init__(self, gait_pattern, balance_controller):
        self.gait_pattern = gait_pattern
        self.balance_controller = balance_controller

    def generate_trajectory_with_gait(self, path, robot_state):
        """Generate trajectory considering gait patterns"""
        trajectory = []

        for i in range(len(path.poses) - 1):
            start_pose = path.poses[i]
            end_pose = path.poses[i + 1]

            # Generate trajectory segment with gait considerations
            segment = self.generate_gait_aware_segment(start_pose, end_pose, robot_state)
            trajectory.extend(segment)

        return trajectory

    def generate_gait_aware_segment(self, start_pose, end_pose, robot_state):
        """Generate trajectory segment with gait awareness"""
        segment = []

        # Calculate required step parameters
        dx = end_pose.pose.position.x - start_pose.pose.position.x
        dy = end_pose.pose.position.y - start_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Determine appropriate step length and number of steps
        num_steps = max(1, int(distance / self.gait_pattern.nominal_step_length))
        step_length = distance / num_steps if num_steps > 0 else 0

        # Generate trajectory points following gait pattern
        for step_idx in range(num_steps):
            # Calculate intermediate pose for this step
            ratio = (step_idx + 1) / num_steps
            intermediate_pose = self.interpolate_pose(start_pose, end_pose, ratio)

            # Apply gait-specific adjustments for balance
            adjusted_pose = self.apply_gait_adjustments(intermediate_pose, step_idx)

            segment.append(adjusted_pose)

        return segment

    def apply_gait_adjustments(self, pose, step_index):
        """Apply gait-specific adjustments for balance"""
        # Add small adjustments based on gait phase
        # This is a simplified example
        gait_phase = step_index % 2  # Alternate between left/right foot

        # Apply small lateral adjustment based on gait phase
        if gait_phase == 0:  # Left foot step
            pose.pose.position.y += 0.05  # Shift slightly right
        else:  # Right foot step
            pose.pose.position.y -= 0.05  # Shift slightly left

        return pose
```

## Humanoid-Specific Navigation Parameters

### Nav2 Configuration for Locomotion

#### Controller Configuration

```yaml
# Humanoid-specific controller configuration
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Higher frequency for balance control
    min_x_velocity_threshold: 0.05  # Minimum velocity for stable walking
    min_y_velocity_threshold: 0.05  # Minimum lateral velocity threshold
    min_theta_velocity_threshold: 0.05  # Minimum angular velocity threshold
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi::SimpleMotionController"
      # Humanoid-specific parameters
      max_linear_speed: 0.4
      max_angular_speed: 0.5
      max_linear_accel: 0.3
      max_angular_accel: 0.4
      min_speed_xy: 0.1
      max_speed_xy: 0.3
      min_speed_theta: 0.1
      max_speed_theta: 0.5
      # Balance-related parameters
      balance_margin: 0.1  # Safety margin for balance
      step_timing_tolerance: 0.1  # Tolerance for step timing
```

#### Costmap Configuration with Locomotion Awareness

```yaml
# Locomotion-aware costmap configuration
local_costmap:
  ros__parameters:
    update_frequency: 10.0
    publish_frequency: 5.0
    global_frame: odom
    robot_base_frame: base_link
    width: 6.0  # Wider for step planning
    height: 6.0
    resolution: 0.025  # Higher resolution for precise footstep planning
    robot_radius: 0.35  # Larger radius for safety
    plugins:
      - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
      - {name: voxel_layer, type: "nav2_costmap_2d::VoxelLayer"}
      - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

    inflation_layer:
      inflation_radius: 0.7  # Larger inflation for bipedal safety
      cost_scaling_factor: 6.0  # Higher scaling for safety
      inflation_kernel_size: 9  # Larger kernel for smoother inflation

global_costmap:
  ros__parameters:
    update_frequency: 2.0
    publish_frequency: 1.0
    global_frame: map
    robot_base_frame: base_link
    width: 100.0
    height: 100.0
    resolution: 0.1  # Lower resolution for global planning
    robot_radius: 0.3  # Robot radius for collision checking
    plugins:
      - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
      - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
      - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

    inflation_layer:
      inflation_radius: 1.0  # Larger inflation for humanoid navigation
      cost_scaling_factor: 4.0
```

## Locomotion-Based Recovery Behaviors

### Balance Recovery Strategies

#### Steady-State Recovery
- **Stance Adjustment**: Adjust foot positions for stability
- **CoM Shifting**: Shift center of mass for balance
- **Arm Movement**: Use arms for balance recovery
- **Step Recovery**: Take recovery steps if needed

#### Dynamic Recovery
- **Emergency Stop**: Stop motion to prevent falls
- **Controlled Fall**: Minimize damage if fall is unavoidable
- **Recovery Gait**: Special gait pattern for recovery
- **Safe Positioning**: Move to safe position before continuing

```python
# Locomotion-based recovery behaviors
class LocomotionRecovery:
    def __init__(self, robot_controller, balance_controller):
        self.robot_controller = robot_controller
        self.balance_controller = balance_controller
        self.recovery_strategies = {
            'stance_adjustment': self.stance_adjustment_recovery,
            'step_recovery': self.step_recovery,
            'emergency_stop': self.emergency_stop
        }

    def execute_recovery(self, recovery_type, current_state):
        """Execute specified recovery behavior"""
        if recovery_type in self.recovery_strategies:
            return self.recovery_strategies[recovery_type](current_state)
        else:
            return False, f"Unknown recovery type: {recovery_type}"

    def stance_adjustment_recovery(self, current_state):
        """Adjust stance to recover balance"""
        # Calculate required foot position adjustments
        current_zmp = self.balance_controller.get_current_zmp()
        current_support_polygon = self.balance_controller.get_current_support_polygon()

        if not self.balance_controller.is_zmp_stable(current_zmp, current_support_polygon):
            # Calculate stable foot positions
            stable_positions = self.balance_controller.calculate_stable_foot_positions(
                current_zmp, current_support_polygon
            )

            # Execute foot movement to stable positions
            success = self.robot_controller.move_feet_to_positions(stable_positions)

            if success:
                return True, "Stance adjustment successful"
            else:
                return False, "Stance adjustment failed"

        return True, "No adjustment needed - already stable"

    def step_recovery(self, current_state):
        """Take recovery step to restore balance"""
        # Calculate safe step location
        current_com = self.balance_controller.get_current_com()
        current_support_polygon = self.balance_controller.get_current_support_polygon()

        # Find safe step location outside current support area
        safe_step_location = self.balance_controller.calculate_safe_step_location(
            current_com, current_support_polygon
        )

        # Execute step to safe location
        success = self.robot_controller.execute_step_to_location(safe_step_location)

        if success:
            return True, "Recovery step successful"
        else:
            return False, "Recovery step failed"

    def emergency_stop(self, current_state):
        """Execute emergency stop to prevent fall"""
        # Execute emergency stop procedure
        # This might involve:
        # - Stopping all motion immediately
        # - Moving to a stable stance
        # - Preparing for possible fall
        self.robot_controller.stop_motion()
        self.robot_controller.move_to_safe_stance()

        return True, "Emergency stop executed"
```

## Terrain-Aware Locomotion

### Adaptive Gait Selection

#### Terrain Classification
- **Flat Ground**: Normal walking gait
- **Rough Terrain**: Adaptive gait for uneven surfaces
- **Sloped Terrain**: Gait adjustments for inclines
- **Narrow Passages**: Careful stepping gait

```python
# Terrain-aware locomotion
class TerrainAwareLocomotion:
    def __init__(self, terrain_classifier, gait_selector):
        self.terrain_classifier = terrain_classifier
        self.gait_selector = gait_selector

    def select_gait_for_terrain(self, terrain_type, navigation_context):
        """Select appropriate gait based on terrain"""
        gait_params = {
            'flat': {
                'step_length': 0.3,
                'step_height': 0.05,
                'step_time': 1.0,
                'balance_margin': 0.1
            },
            'rough': {
                'step_length': 0.2,
                'step_height': 0.15,
                'step_time': 1.5,
                'balance_margin': 0.15
            },
            'sloped': {
                'step_length': 0.25,
                'step_height': 0.1,
                'step_time': 1.2,
                'balance_margin': 0.12
            },
            'narrow': {
                'step_length': 0.15,
                'step_width': 0.1,
                'step_time': 1.8,
                'balance_margin': 0.2
            }
        }

        if terrain_type in gait_params:
            return gait_params[terrain_type]
        else:
            # Default to conservative gait for unknown terrain
            return gait_params['rough']
```

## Performance Optimization

### Computational Considerations

#### Real-time Requirements
- **Balance Control Frequency**: High-frequency balance control (100-200 Hz)
- **Step Planning**: Moderate frequency for step planning (10-20 Hz)
- **Path Planning**: Lower frequency for global path planning (1-5 Hz)
- **Trajectory Generation**: High frequency for trajectory updates (50-100 Hz)

#### Optimization Strategies
- **Pre-computed Gaits**: Pre-compute common gait patterns
- **Caching**: Cache locomotion parameters
- **Simplified Models**: Use simplified models for real-time control
- **Hierarchical Control**: Multi-level control architecture

## Best Practices

### Configuration Guidelines

1. **Start Conservative**: Begin with conservative locomotion parameters
2. **Validate Continuously**: Continuously validate balance and stability
3. **Test Gradually**: Test with increasingly complex locomotion patterns
4. **Monitor Performance**: Monitor computational performance
5. **Safety First**: Prioritize safety over efficiency

### Implementation Tips

- **Modular Design**: Keep locomotion components modular
- **Parameter Tuning**: Make parameters easily tunable
- **Fallback Systems**: Implement robust fallback behaviors
- **Logging**: Log locomotion data for analysis
- **Visualization**: Visualize gait patterns and balance data

## Next Steps

With humanoid locomotion constraints understood, you're ready to explore:

- Obstacle avoidance techniques specific to bipedal robots
- Advanced path planning with dynamic constraints
- Integration with Isaac Sim for validation
- Stability considerations in navigation

Continue to the next section to learn about obstacle avoidance for humanoid robots.