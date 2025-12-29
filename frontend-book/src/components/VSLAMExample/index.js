import React, { useState, useEffect } from 'react';

const VSLAMExample = () => {
  const [isRunning, setIsRunning] = useState(false);
  const [status, setStatus] = useState('Ready');
  const [mapData, setMapData] = useState(null);
  const [poseData, setPoseData] = useState({ x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 });

  // Simulate VSLAM processing
  const simulateVSLAM = () => {
    if (!isRunning) return;

    // Simulate pose estimation
    const newPose = {
      x: poseData.x + (Math.random() - 0.5) * 0.1,
      y: poseData.y + (Math.random() - 0.5) * 0.1,
      z: poseData.z + (Math.random() - 0.5) * 0.05,
      roll: poseData.roll + (Math.random() - 0.5) * 0.01,
      pitch: poseData.pitch + (Math.random() - 0.5) * 0.01,
      yaw: poseData.yaw + (Math.random() - 0.5) * 0.02
    };
    setPoseData(newPose);

    // Simulate map building
    if (Math.random() > 0.7) {
      const newMapPoint = {
        id: Date.now(),
        x: newPose.x + (Math.random() - 0.5) * 2,
        y: newPose.y + (Math.random() - 0.5) * 2,
        z: newPose.z + (Math.random() - 0.5) * 1,
        confidence: Math.random()
      };

      setMapData(prev => {
        const updated = prev ? [...prev, newMapPoint] : [newMapPoint];
        // Limit to 100 points for performance
        return updated.slice(-100);
      });
    }

    setStatus(`Processing... Pose: (${newPose.x.toFixed(2)}, ${newPose.y.toFixed(2)}, ${newPose.yaw.toFixed(2)})`);
  };

  useEffect(() => {
    let interval;
    if (isRunning) {
      interval = setInterval(simulateVSLAM, 100); // 10 Hz simulation
    }
    return () => clearInterval(interval);
  }, [isRunning, poseData]);

  const toggleSimulation = () => {
    setIsRunning(!isRunning);
    if (!isRunning) {
      setStatus('Running VSLAM simulation...');
    } else {
      setStatus('VSLAM stopped');
    }
  };

  const resetSimulation = () => {
    setIsRunning(false);
    setStatus('Ready');
    setMapData(null);
    setPoseData({ x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 });
  };

  return (
    <div className="vslam-example">
      <h3>Isaac ROS VSLAM Example</h3>
      <p>This example demonstrates Isaac ROS hardware-accelerated VSLAM capabilities.</p>

      <div className="simulation-controls">
        <button onClick={toggleSimulation}>
          {isRunning ? 'Stop VSLAM' : 'Start VSLAM'}
        </button>
        <button onClick={resetSimulation}>
          Reset Simulation
        </button>
      </div>

      <div className="status-panel">
        <h4>Status: {status}</h4>
        <div className="pose-data">
          <h5>Current Pose:</h5>
          <p>X: {poseData.x.toFixed(3)}m, Y: {poseData.y.toFixed(3)}m, Z: {poseData.z.toFixed(3)}m</p>
          <p>Roll: {poseData.roll.toFixed(3)}rad, Pitch: {poseData.pitch.toFixed(3)}rad, Yaw: {poseData.yaw.toFixed(3)}rad</p>
        </div>
        <div className="map-info">
          <h5>Map Data:</h5>
          <p>Points: {mapData ? mapData.length : 0}</p>
        </div>
      </div>

      <div className="visualization-area">
        <h4>VSLAM Visualization</h4>
        <div className="map-visualization">
          {mapData && mapData.length > 0 ? (
            <div className="map-canvas">
              {/* Simple visualization of map points */}
              <svg width="400" height="400" viewBox="-10 -10 20 20">
                {/* Draw map points */}
                {mapData.map((point, index) => (
                  <circle
                    key={point.id}
                    cx={point.x * 2} // Scale for visualization
                    cy={point.y * 2}
                    r={0.1}
                    fill="blue"
                    opacity={point.confidence}
                  />
                ))}

                {/* Draw current robot position */}
                <circle
                  cx={poseData.x * 2}
                  cy={poseData.y * 2}
                  r={0.3}
                  fill="red"
                  stroke="black"
                  strokeWidth="0.05"
                />

                {/* Draw robot orientation */}
                <line
                  x1={poseData.x * 2}
                  y1={poseData.y * 2}
                  x2={poseData.x * 2 + Math.cos(poseData.yaw) * 1}
                  y2={poseData.y * 2 + Math.sin(poseData.yaw) * 1}
                  stroke="red"
                  strokeWidth="0.1"
                />
              </svg>
            </div>
          ) : (
            <p>No map data yet. Start VSLAM to begin mapping.</p>
          )}
        </div>
      </div>

      <div className="code-example">
        <h4>Isaac ROS VSLAM Integration Example:</h4>
        <pre>
{`# Example Isaac ROS VSLAM launch configuration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Isaac ROS Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam',
            parameters=[{
                'enable_rectified_pose': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_odom_tf': True,
                'enable_observations_view': True,
                'enable_slam_visualization': True
            }],
            remappings=[
                ('/visual_slam/image', '/camera/image_rect_color'),
                ('/visual_slam/camera_info', '/camera/camera_info'),
            ]
        )
    ])`}
        </pre>
      </div>

      <div className="key-features">
        <h4>Key VSLAM Features Demonstrated:</h4>
        <ul>
          <li>Real-time visual-inertial SLAM with GPU acceleration</li>
          <li>6-DOF pose estimation from monocular camera input</li>
          <li>Feature-based mapping and localization</li>
          <li>Loop closure detection and correction</li>
          <li>Robust tracking under motion blur and lighting changes</li>
        </ul>
      </div>
    </div>
  );
};

export default VSLAMExample;