import React, { useState, useEffect } from 'react';

const Nav2Example = () => {
  const [isRunning, setIsRunning] = useState(false);
  const [status, setStatus] = useState('Ready');
  const [robotPosition, setRobotPosition] = useState({ x: 0, y: 0, theta: 0 });
  const [goalPosition, setGoalPosition] = useState({ x: 5, y: 5 });
  const [path, setPath] = useState([]);
  const [obstacles, setObstacles] = useState([
    { id: 1, x: 2, y: 2, radius: 0.5 },
    { id: 2, x: 3, y: 4, radius: 0.3 },
    { id: 3, x: 1, y: 3, radius: 0.4 }
  ]);
  const [navigationMode, setNavigationMode] = useState('idle'); // idle, planning, executing, paused

  // Simulate humanoid navigation
  const simulateNavigation = () => {
    if (!isRunning || navigationMode !== 'executing') return;

    // Simple simulation: move robot along the path
    if (path.length > 0) {
      // Move robot towards next waypoint in path
      const nextWaypoint = path[0];
      const dx = nextWaypoint.x - robotPosition.x;
      const dy = nextWaypoint.y - robotPosition.y;
      const distance = Math.sqrt(dx * dx + dy * dy);

      if (distance < 0.1) {
        // Reached waypoint, move to next one
        setPath(prevPath => prevPath.slice(1));
        if (path.length === 1) {
          // Reached goal
          setNavigationMode('idle');
          setStatus('Navigation completed successfully!');
          return;
        }
      } else {
        // Move towards waypoint
        const speed = 0.05; // 5cm per step
        const moveDistance = Math.min(speed, distance);
        const newX = robotPosition.x + (dx / distance) * moveDistance;
        const newY = robotPosition.y + (dy / distance) * moveDistance;
        const newTheta = Math.atan2(dy, dx);

        setRobotPosition({ x: newX, y: newY, theta: newTheta });
        setStatus(`Navigating... Distance to goal: ${distance.toFixed(2)}m`);
      }
    }
  };

  useEffect(() => {
    let interval;
    if (isRunning) {
      interval = setInterval(simulateNavigation, 100); // 10 Hz simulation
    }
    return () => clearInterval(interval);
  }, [isRunning, robotPosition, path, navigationMode]);

  const startNavigation = () => {
    if (navigationMode === 'idle') {
      // Plan path (simulated)
      planPath();
      setNavigationMode('planning');
      setStatus('Planning path...');
    } else if (navigationMode === 'planning') {
      // Start executing the planned path
      setNavigationMode('executing');
      setStatus('Executing navigation...');
      setIsRunning(true);
    }
  };

  const planPath = () => {
    // Simulate path planning with obstacle avoidance
    // In a real implementation, this would call Nav2 path planning
    const newPath = [];

    // Simple path planning algorithm simulation
    // Start from robot position
    newPath.push({ x: robotPosition.x, y: robotPosition.y });

    // Add intermediate waypoints avoiding obstacles
    for (let i = 0; i < 10; i++) {
      const progress = i / 10;
      const targetX = robotPosition.x + (goalPosition.x - robotPosition.x) * progress;
      const targetY = robotPosition.y + (goalPosition.y - robotPosition.y) * progress;

      // Add some obstacle avoidance logic
      let adjustedX = targetX;
      let adjustedY = targetY;

      // Check for obstacles and adjust path if needed
      for (const obstacle of obstacles) {
        const distToObstacle = Math.sqrt(
          Math.pow(adjustedX - obstacle.x, 2) + Math.pow(adjustedY - obstacle.y, 2)
        );

        if (distToObstacle < obstacle.radius + 0.5) { // Safety margin
          // Adjust path to go around obstacle
          adjustedX += (Math.random() - 0.5) * 0.5;
          adjustedY += (Math.random() - 0.5) * 0.5;
        }
      }

      newPath.push({ x: adjustedX, y: adjustedY });
    }

    // Add goal position
    newPath.push({ x: goalPosition.x, y: goalPosition.y });

    setPath(newPath);
    setNavigationMode('planning');
  };

  const stopNavigation = () => {
    setIsRunning(false);
    setNavigationMode('idle');
    setStatus('Navigation stopped');
  };

  const resetSimulation = () => {
    setIsRunning(false);
    setNavigationMode('idle');
    setRobotPosition({ x: 0, y: 0, theta: 0 });
    setPath([]);
    setStatus('Ready');
  };

  const updateGoal = (e) => {
    const [x, y] = e.currentTarget.value.split(',').map(Number);
    if (!isNaN(x) && !isNaN(y)) {
      setGoalPosition({ x, y });
    }
  };

  return (
    <div className="nav2-example">
      <h3>Nav2 Humanoid Robot Navigation Example</h3>
      <p>This example demonstrates Nav2 path planning and execution for humanoid robots with obstacle avoidance.</p>

      <div className="simulation-controls">
        <button onClick={startNavigation} disabled={isRunning || navigationMode === 'executing'}>
          {navigationMode === 'idle' ? 'Plan & Start Navigation' :
           navigationMode === 'planning' ? 'Execute Navigation' : 'Start Navigation'}
        </button>
        <button onClick={stopNavigation} disabled={!isRunning}>
          Stop Navigation
        </button>
        <button onClick={resetSimulation}>
          Reset Simulation
        </button>
      </div>

      <div className="status-panel">
        <h4>Status: {status}</h4>
        <div className="robot-info">
          <h5>Robot Position:</h5>
          <p>X: {robotPosition.x.toFixed(2)}m, Y: {robotPosition.y.toFixed(2)}m, θ: {robotPosition.theta.toFixed(2)}rad</p>
        </div>
        <div className="goal-info">
          <h5>Goal Position:</h5>
          <p>X: {goalPosition.x.toFixed(2)}m, Y: {goalPosition.y.toFixed(2)}m</p>
        </div>
        <div className="path-info">
          <h5>Path Information:</h5>
          <p>Waypoints: {path.length}</p>
          <p>Mode: {navigationMode}</p>
        </div>
      </div>

      <div className="visualization-area">
        <h4>Navigation Visualization</h4>
        <div className="navigation-map">
          <svg width="500" height="500" viewBox="0 0 10 10">
            {/* Grid background */}
            <defs>
              <pattern id="grid" width="0.5" height="0.5" patternUnits="userSpaceOnUse">
                <path d="M 0.5 0 L 0 0 0 0.5" fill="none" stroke="#e0e0e0" strokeWidth="0.01"/>
              </pattern>
            </defs>
            <rect width="10" height="10" fill="url(#grid)" />

            {/* Obstacles */}
            {obstacles.map(obstacle => (
              <circle
                key={obstacle.id}
                cx={obstacle.x}
                cy={obstacle.y}
                r={obstacle.radius}
                fill="red"
                opacity="0.6"
              />
            ))}

            {/* Planned path */}
            {path.length > 1 && (
              <polyline
                points={path.map(p => `${p.x},${p.y}`).join(' ')}
                fill="none"
                stroke="blue"
                strokeWidth="0.05"
                opacity="0.7"
              />
            )}

            {/* Robot */}
            <circle
              cx={robotPosition.x}
              cy={robotPosition.y}
              r="0.2"
              fill="green"
              stroke="black"
              strokeWidth="0.02"
            />
            {/* Robot orientation indicator */}
            <line
              x1={robotPosition.x}
              y1={robotPosition.y}
              x2={robotPosition.x + Math.cos(robotPosition.theta) * 0.3}
              y2={robotPosition.y + Math.sin(robotPosition.theta) * 0.3}
              stroke="black"
              strokeWidth="0.03"
            />

            {/* Goal */}
            <circle
              cx={goalPosition.x}
              cy={goalPosition.y}
              r="0.15"
              fill="yellow"
              stroke="black"
              strokeWidth="0.02"
            />
            <text
              x={goalPosition.x}
              y={goalPosition.y - 0.25}
              textAnchor="middle"
              fontSize="0.3"
              fill="black"
            >
              Goal
            </text>
          </svg>
        </div>
      </div>

      <div className="configuration-panel">
        <h4>Configuration</h4>
        <div className="input-group">
          <label>Goal Position (x,y):</label>
          <input
            type="text"
            placeholder="5,5"
            defaultValue={`${goalPosition.x},${goalPosition.y}`}
            onChange={updateGoal}
          />
        </div>
        <div className="info-box">
          <h5>Humanoid Navigation Parameters:</h5>
          <ul>
            <li>Max step length: 0.3m</li>
            <li>Max step height: 0.15m</li>
            <li>Balance margin: 0.1m</li>
            <li>Foot separation: 0.25m</li>
            <li>Max linear speed: 0.3 m/s</li>
            <li>Max angular speed: 0.5 rad/s</li>
          </ul>
        </div>
      </div>

      <div className="code-example">
        <h4>Nav2 Humanoid Navigation Launch Example:</h4>
        <pre>
{`# Example Nav2 humanoid navigation launch file
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    return LaunchDescription([
        # Navigation Server
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[
                {'use_sim_time': True},
                {'global_frame': 'map'},
                {'robot_base_frame': 'base_link'},
                {'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_follow_path_action_bt_node',
                    'nav2_back_up_action_bt_node',
                    'nav2_spin_action_bt_node',
                    'nav2_wait_action_bt_node',
                    'nav2_clear_costmap_service_bt_node',
                    'nav2_is_stuck_condition_bt_node',
                    'nav2_goal_reached_condition_bt_node',
                    'nav2_goal_updated_condition_bt_node',
                    'nav2_initial_pose_received_condition_bt_node',
                    'nav2_reinitialize_global_localization_service_bt_node',
                    'nav2_rate_controller_bt_node',
                    'nav2_distance_controller_bt_node',
                    'nav2_speed_controller_bt_node',
                    'nav2_truncate_path_action_bt_node',
                    'nav2_goal_updater_node_bt_node',
                    'nav2_recovery_node_bt_node',
                    'nav2_pipeline_sequence_bt_node',
                    'nav2_round_robin_node_bt_node',
                    'nav2_transform_available_condition_bt_node',
                    'nav2_time_expired_condition_bt_node',
                    'nav2_path_expiring_timer_condition',
                    'nav2_distance_traveled_condition_bt_node',
                    'nav2_single_trigger_bt_node',
                    'nav2_is_battery_low_condition_bt_node',
                    'nav2_navigate_through_poses_action_bt_node',
                    'nav2_navigate_to_pose_action_bt_node',
                    'nav2_remove_passed_goals_action_bt_node',
                    'nav2_planner_selector_bt_node',
                    'nav2_controller_selector_bt_node',
                    'nav2_goal_checker_selector_bt_node'
                ]}
            ]
        ),

        # Local Planner for Humanoid
        Node(
            package='nav2_dwb_controller',
            executable='dwb_controller',
            name='dwb_controller',
            parameters=[
                {'use_sim_time': True},
                {'min_vel_x': 0.05},      # Minimum forward speed for stable walking
                {'max_vel_x': 0.3},       # Maximum forward speed
                {'min_vel_y': -0.1},      # Minimum lateral speed
                {'max_vel_y': 0.1},       # Maximum lateral speed
                {'max_vel_theta': 0.5},   # Maximum angular velocity
                {'acc_lim_x': 0.3},       # Acceleration limits for stability
                {'acc_lim_theta': 0.3},
                {'decel_lim_x': -0.3},
                {'decel_lim_theta': -0.3}
            ]
        )
    ])`}
        </pre>
      </div>

      <div className="key-features">
        <h4>Key Humanoid Navigation Features:</h4>
        <ul>
          <li>Bipedal-specific path planning with step constraints</li>
          <li>Balance-aware obstacle avoidance</li>
          <li>Footstep planning integration</li>
          <li>Dynamic obstacle handling with pedestrian awareness</li>
          <li>Stability monitoring during navigation</li>
          <li>Recovery behaviors for navigation failures</li>
        </ul>
      </div>
    </div>
  );
};

export default Nav2Example;