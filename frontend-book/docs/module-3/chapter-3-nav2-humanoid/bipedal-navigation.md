# Bipedal Navigation in Nav2

## Overview

Bipedal navigation presents unique challenges compared to traditional wheeled robot navigation. Humanoid robots must maintain balance while navigating, plan foot placements carefully, and manage complex kinematic constraints. This section covers how to adapt Nav2 for bipedal humanoid robots, addressing the specific requirements of walking locomotion.

## Key Differences from Wheeled Navigation

### Dynamic Balance Requirements

Unlike wheeled robots that maintain static stability, humanoid robots must maintain dynamic balance:

#### Zero Moment Point (ZMP) Considerations
- **ZMP Trajectory Planning**: Plan ZMP trajectory within support polygon
- **Balance Maintenance**: Continuously adjust to maintain balance
- **Footstep Timing**: Coordinate steps with balance requirements
- **Center of Mass (CoM) Control**: Manage CoM position during movement

#### Gait Pattern Integration
- **Walking Patterns**: Implement stable walking gaits (e.g., inverted pendulum)
- **Step Sequencing**: Plan alternating foot steps
- **Double Support Phase**: Manage phases where both feet are on ground
- **Single Support Phase**: Handle phases with one foot on ground

```python
# Bipedal navigation constraints
class BipedalNavigationConstraints:
    def __init__(self):
        self.max_step_height = 0.15  # 15 cm maximum step height
        self.max_step_length = 0.30  # 30 cm maximum step length
        self.max_step_width = 0.25   # 25 cm maximum step width
        self.min_step_time = 0.5     # 0.5 seconds minimum step time
        self.balance_margin = 0.1    # 10 cm balance safety margin

    def validate_path_for_bipedal(self, path):
        """Validate that a path is feasible for bipedal navigation"""
        for i in range(len(path) - 1):
            step_vector = [
                path[i+1].pose.position.x - path[i].pose.position.x,
                path[i+1].pose.position.y - path[i].pose.position.y
            ]

            step_length = math.sqrt(step_vector[0]**2 + step_vector[1]**2)

            if step_length > self.max_step_length:
                return False, f"Step from {i} to {i+1} too long: {step_length:.2f}m"

            # Check step height constraints
            step_height = abs(path[i+1].pose.position.z - path[i].pose.position.z)
            if step_height > self.max_step_height:
                return False, f"Step from {i} to {i+1} too high: {step_height:.2f}m"

        return True, "Path is feasible for bipedal navigation"
```

### Footstep Planning Integration

#### Path to Footsteps Conversion
- **Footstep Generation**: Convert continuous path to discrete foot placements
- **Support Polygon**: Ensure each step maintains balance
- **Footprint Planning**: Plan where each foot should go
- **Timing Coordination**: Coordinate footstep timing with path execution

```python
# Footstep planning
class FootstepPlanner:
    def __init__(self, robot_config):
        self.robot_config = robot_config
        self.foot_separation = robot_config['foot_separation']  # Distance between feet
        self.step_width = robot_config['step_width']           # Width of step
        self.step_length = robot_config['step_length']         # Length of step

    def plan_footsteps(self, path, start_pose):
        """Plan foot placements along a path"""
        footsteps = []

        # Start with current foot positions
        left_foot = self.calculate_left_foot_position(start_pose)
        right_foot = self.calculate_right_foot_position(start_pose)

        current_left_support = True  # Start with left foot support

        for i in range(len(path) - 1):
            # Calculate desired step direction
            dx = path[i+1].pose.position.x - path[i].pose.position.x
            dy = path[i+1].pose.position.y - path[i].pose.position.y
            step_direction = math.atan2(dy, dx)

            # Calculate next foot position based on current support foot
            if current_left_support:
                # Move right foot
                next_right_x = path[i].pose.position.x + self.step_length * math.cos(step_direction)
                next_right_y = path[i].pose.position.y + self.step_length * math.sin(step_direction)
                next_right_z = path[i].pose.position.z  # Assume flat terrain

                footsteps.append({
                    'foot': 'right',
                    'position': [next_right_x, next_right_y, next_right_z],
                    'timestamp': i * self.robot_config['step_duration']
                })

                current_left_support = False
            else:
                # Move left foot
                next_left_x = path[i].pose.position.x + self.step_length * math.cos(step_direction)
                next_left_y = path[i].pose.position.y + self.step_length * math.sin(step_direction)
                next_left_z = path[i].pose.position.z

                footsteps.append({
                    'foot': 'left',
                    'position': [next_left_x, next_left_y, next_left_z],
                    'timestamp': i * self.robot_config['step_duration']
                })

                current_left_support = True

        return footsteps

    def calculate_left_foot_position(self, pose):
        """Calculate initial left foot position"""
        # Calculate based on robot's current pose
        pass

    def calculate_right_foot_position(self, pose):
        """Calculate initial right foot position"""
        # Calculate based on robot's current pose
        pass
```

## Nav2 Configuration for Bipedal Navigation

### Specialized Costmap Configuration

Bipedal robots require different costmap considerations:

```yaml
# Bipedal-specific costmap configuration
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  transform_tolerance: 0.5
  width: 50
  height: 50
  resolution: 0.05  # Higher resolution for precise footstep planning

  # Bipedal-specific footprint
  footprint: [[0.3, 0.25], [0.3, -0.25], [-0.2, -0.25], [-0.2, 0.25]]
  footprint_padding: 0.02  # Slightly larger padding for safety

  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

  inflation_layer:
    inflation_radius: 0.6  # Larger inflation for bipedal safety
    cost_scaling_factor: 5.0  # Higher scaling for safety
    inflation_kernel_size: 7  # Larger kernel for smoother inflation

obstacle_layer:
  enabled: true
  observation_sources: scan
  scan:
    topic: /laser_scan
    max_obstacle_height: 2.0  # Consider obstacles up to 2m high
    clearing: true
    marking: true
    data_type: LaserScan
    obstacle_range: 3.0
    raytrace_range: 4.0
```

### Local Planner Configuration

Local planners need to consider bipedal constraints:

```yaml
# Bipedal-specific local planner configuration
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 5.0
  transform_tolerance: 0.5
  width: 5.0
  height: 5.0
  resolution: 0.025  # Higher resolution for precise navigation
  robot_radius: 0.3  # Radius for collision checking

  plugins:
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: voxel_layer, type: "nav2_costmap_2d::VoxelLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

local_planner:
  plugin: "dwb_core::DWBLocalPlanner"

  # Bipedal-specific parameters
  debug_trajectory_details: true
  min_vel_x: 0.05  # Minimum forward speed for stable walking
  max_vel_x: 0.3   # Maximum forward speed
  min_vel_y: -0.1  # Minimum lateral speed
  max_vel_y: 0.1   # Maximum lateral speed
  max_vel_theta: 0.5  # Maximum angular velocity
  min_speed_xy: 0.1   # Minimum combined XY speed
  max_speed_xy: 0.3   # Maximum combined XY speed
  min_speed_theta: 0.1 # Minimum angular speed

  # Acceleration limits for stable walking
  acc_lim_x: 0.5
  acc_lim_y: 0.2
  acc_lim_theta: 0.5

  # Deceleration limits
  decel_lim_x: -0.5
  decel_lim_y: -0.2
  decel_lim_theta: -0.5
```

## Bipedal-Specific Path Planning

### Path Smoothing for Walking

Paths need to be smoothed appropriately for bipedal locomotion:

```python
# Path smoothing for bipedal robots
class BipedalPathSmoother:
    def __init__(self):
        self.max_curvature = 0.5  # Maximum path curvature
        self.min_distance = 0.1   # Minimum distance between waypoints

    def smooth_path(self, path):
        """Smooth path for bipedal navigation"""
        if len(path.poses) < 3:
            return path

        # Convert to numpy array for processing
        points = []
        for pose in path.poses:
            points.append([pose.pose.position.x, pose.pose.position.y])

        points = np.array(points)

        # Apply smoothing while respecting curvature constraints
        smoothed_points = self.apply_curvature_constrained_smoothing(points)

        # Convert back to path format
        smoothed_path = Path()
        smoothed_path.header = path.header

        for point in smoothed_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0  # Assume flat terrain
            pose_stamped.pose.orientation.w = 1.0  # No rotation
            smoothed_path.poses.append(pose_stamped)

        return smoothed_path

    def apply_curvature_constrained_smoothing(self, points):
        """Apply smoothing with curvature constraints"""
        # Use a smoothing algorithm that respects curvature limits
        # This is a simplified example - real implementation would be more complex
        from scipy.interpolate import UnivariateSpline

        if len(points) < 4:
            return points

        # Parameterize the curve
        t = np.linspace(0, 1, len(points))

        # Fit splines to x and y coordinates
        spline_x = UnivariateSpline(t, points[:, 0], s=0.1)
        spline_y = UnivariateSpline(t, points[:, 1], s=0.1)

        # Generate smoothed points
        t_new = np.linspace(0, 1, len(points) * 2)  # More points for smoother path
        x_smooth = spline_x(t_new)
        y_smooth = spline_y(t_new)

        return np.column_stack([x_smooth, y_smooth])
```

### Step-Aware Path Planning

```python
# Step-aware path planning
class StepAwarePathPlanner:
    def __init__(self, step_constraints):
        self.step_constraints = step_constraints
        self.footstep_planner = FootstepPlanner(step_constraints)

    def plan_path_with_steps(self, start, goal, map):
        """Plan path with consideration for discrete steps"""
        # First, plan a basic path using global planner
        basic_path = self.plan_basic_path(start, goal, map)

        # Then, validate and adjust for step constraints
        validated_path = self.validate_for_step_constraints(basic_path)

        # Finally, generate footstep plan
        footsteps = self.footstep_planner.plan_footsteps(validated_path, start)

        return validated_path, footsteps

    def validate_for_step_constraints(self, path):
        """Validate path for step constraints and adjust if needed"""
        validated_path = []

        i = 0
        while i < len(path.poses):
            current_point = path.poses[i]
            validated_path.append(current_point)

            # Look ahead to find next valid point based on step constraints
            j = i + 1
            while j < len(path.poses):
                next_point = path.poses[j]

                step_distance = self.calculate_step_distance(current_point, next_point)

                if step_distance <= self.step_constraints['max_step_length']:
                    # This point is valid, add it to validated path
                    validated_path.append(next_point)
                    i = j
                    break
                else:
                    # This step is too long, need intermediate points
                    intermediate = self.interpolate_step(current_point, next_point)
                    validated_path.append(intermediate)
                    i = j  # Continue from next point
                    break

                j += 1

            if j >= len(path.poses):
                # Reached end of path
                break

        # Create new path with validated points
        result_path = Path()
        result_path.header = path.header
        result_path.poses = validated_path

        return result_path

    def calculate_step_distance(self, pose1, pose2):
        """Calculate step distance between two poses"""
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def interpolate_step(self, start_pose, end_pose):
        """Interpolate an intermediate step if direct step is too long"""
        # Calculate midpoint
        mid_x = (start_pose.pose.position.x + end_pose.pose.position.x) / 2.0
        mid_y = (start_pose.pose.position.y + end_pose.pose.position.y) / 2.0
        mid_z = (start_pose.pose.position.z + end_pose.pose.position.z) / 2.0

        # Create intermediate pose
        intermediate = PoseStamped()
        intermediate.header = start_pose.header
        intermediate.pose.position.x = mid_x
        intermediate.pose.position.y = mid_y
        intermediate.pose.position.z = mid_z
        intermediate.pose.orientation = start_pose.pose.orientation

        return intermediate
```

## Balance-Aware Navigation

### Integration with Balance Control

Navigation must be coordinated with balance control systems:

```python
# Balance-aware navigation
class BalanceAwareNavigator:
    def __init__(self, balance_controller):
        self.balance_controller = balance_controller
        self.navigation_active = False
        self.safety_thresholds = {
            'max_tilt': 15.0,  # Maximum acceptable tilt in degrees
            'min_balance_score': 0.7  # Minimum balance quality score
        }

    def navigate_with_balance(self, path, goal):
        """Navigate while monitoring and maintaining balance"""
        self.navigation_active = True

        # Initialize footstep execution
        footstep_executor = self.initialize_footstep_execution(path)

        for step in footstep_executor:
            # Check balance before executing step
            balance_status = self.balance_controller.get_balance_status()

            if not self.is_balance_acceptable(balance_status):
                # Stop navigation and recover balance
                self.pause_navigation()
                self.recover_balance(balance_status)

                if not self.is_balance_recovered():
                    # Unable to recover balance, abort navigation
                    return False, "Balance recovery failed"

            # Execute the footstep
            success = self.execute_footstep(step)

            if not success:
                # Handle step execution failure
                recovery_success = self.execute_recovery_behavior()
                if not recovery_success:
                    return False, "Step execution failed with no recovery"

        # Navigation completed successfully
        self.navigation_active = False
        return True, "Navigation completed successfully"

    def is_balance_acceptable(self, balance_status):
        """Check if balance is acceptable for navigation"""
        tilt_angle = self.calculate_tilt_angle(balance_status)

        return (abs(tilt_angle) < self.safety_thresholds['max_tilt'] and
                balance_status['quality_score'] > self.safety_thresholds['min_balance_score'])

    def recover_balance(self, balance_status):
        """Recover robot balance"""
        # Implement balance recovery strategy
        # This might involve:
        # - Adjusting foot placement
        # - Modifying step timing
        # - Using arm movements for balance
        # - Stopping motion temporarily
        pass

    def execute_recovery_behavior(self):
        """Execute navigation recovery behavior"""
        # Implement recovery behavior
        # This could be a predefined set of movements to recover from navigation failure
        pass
```

## Humanoid-Specific Navigation Behaviors

### Specialized Behavior Trees

Bipedal navigation requires specialized behavior trees:

```xml
<!-- Bipedal-specific behavior tree -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="NavigateToPoseBipedal">
      <!-- Check if robot is in stable state -->
      <CheckBalanceStability/>

      <!-- Plan path with step constraints -->
      <Fallback name="GlobalPlanBipedal">
        <Sequence name="ComputeBipedalPath">
          <IsPathValidBipedal/>
          <ComputePathToPoseBipedal/>
          <ValidateStepConstraints/>
        </Sequence>
        <ReactiveFallback name="GlobalPlanRecovery">
          <ReactiveSequence name="ClearGlobalCostmap">
            <ClearEntirelyCostmap name="global_clear" service_name="global_costmap/clear_entirely"/>
          </ReactiveSequence>
          <ReactiveSequence name="ComputePathRetry">
            <IsPathValidBipedal/>
            <ComputePathToPoseBipedal/>
            <ValidateStepConstraints/>
          </ReactiveSequence>
        </ReactiveFallback>
      </Fallback>

      <!-- Follow path with footstep planning -->
      <Sequence name="FollowPathBipedal">
        <CheckBalanceStability/>
        <GenerateFootsteps/>
        <FollowFootstepPath/>
        <IsGoalReachedBipedal/>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
```

### Recovery Behaviors

Special recovery behaviors for bipedal robots:

```python
# Bipedal recovery behaviors
class BipedalRecoveryBehaviors:
    def __init__(self, robot_controller):
        self.robot_controller = robot_controller

    def balance_recovery_spin(self):
        """Recovery behavior: attempt to regain balance by adjusting stance"""
        # Move to a more stable stance
        # This might involve widening foot separation
        # or adjusting center of mass position
        current_stance = self.robot_controller.get_current_stance()
        stable_stance = self.calculate_stable_stance(current_stance)

        return self.robot_controller.move_to_stance(stable_stance)

    def step_back_recovery(self):
        """Recovery behavior: take a step back to safer position"""
        # Calculate safe step back position
        current_pose = self.robot_controller.get_current_pose()
        safe_pose = self.calculate_safe_backup_position(current_pose)

        return self.robot_controller.execute_step_to_pose(safe_pose)

    def widen_stance_recovery(self):
        """Recovery behavior: widen stance for better stability"""
        # Adjust foot positions to create wider support polygon
        current_feet_poses = self.robot_controller.get_current_feet_poses()
        wider_feet_poses = self.calculate_wider_stance(current_feet_poses)

        return self.robot_controller.move_feet_to_poses(wider_feet_poses)

    def calculate_stable_stance(self, current_stance):
        """Calculate a more stable stance"""
        # Implementation to calculate stable stance
        # This would consider ZMP, CoM, and support polygon
        pass
```

## Performance Considerations

### Computational Requirements

Bipedal navigation has higher computational requirements:

#### Planning Complexity
- **Footstep Planning**: Additional computational overhead for step planning
- **Balance Monitoring**: Continuous balance computation
- **Path Validation**: Validation against step constraints
- **Dynamic Replanning**: More frequent replanning due to constraints

#### Real-time Requirements
- **Control Frequency**: Higher control frequency for balance
- **Sensor Processing**: Fast processing of balance sensors
- **Path Execution**: Precise timing for step execution
- **Recovery Planning**: Fast recovery behavior planning

## Integration with Other Systems

### Coordination with Perception

Bipedal navigation must coordinate with perception systems:

```python
# Integration with perception
class PerceptionIntegratedNavigator:
    def __init__(self, perception_system, navigation_system):
        self.perception = perception_system
        self.navigation = navigation_system
        self.terrain_analyzer = TerrainAnalyzer()

    def navigate_with_terrain_awareness(self, goal):
        """Navigate with awareness of terrain characteristics"""
        # Analyze terrain ahead
        terrain_info = self.perception.get_terrain_ahead(goal)

        # Adjust navigation parameters based on terrain
        if terrain_info['roughness'] > 0.1:  # Rough terrain
            self.navigation.adjust_step_height(terrain_info['roughness'])
            self.navigation.reduce_speed(0.5)  # Reduce to 50% speed

        if terrain_info['slope'] > 15:  # Steep slope (>15 degrees)
            self.navigation.use_climbing_gait()

        # Plan and execute navigation
        return self.navigation.navigate(goal)
```

## Best Practices

### Configuration Guidelines

1. **Start Conservative**: Begin with conservative step constraints
2. **Validate Continuously**: Continuously validate balance during navigation
3. **Plan for Recovery**: Always have recovery behaviors ready
4. **Test Incrementally**: Test navigation in simple environments first
5. **Monitor Performance**: Monitor computational performance for real-time constraints

### Safety Considerations

- **Balance Margins**: Maintain safety margins for balance
- **Step Validation**: Validate each step before execution
- **Emergency Stops**: Implement emergency stop capabilities
- **Recovery Behaviors**: Maintain multiple recovery options
- **Terrain Assessment**: Assess terrain before navigation

## Next Steps

With bipedal navigation fundamentals understood, you're ready to explore:

- Humanoid locomotion patterns and gait generation
- Obstacle avoidance techniques specific to bipedal robots
- Advanced path planning with dynamic constraints
- Integration with Isaac Sim for validation

Continue to the next section to learn about humanoid locomotion in navigation.