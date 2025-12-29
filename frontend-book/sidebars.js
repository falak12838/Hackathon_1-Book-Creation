// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Single course sidebar for Physical AI & Humanoid Robotics
  courseSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'category',
          label: 'Module 1: ROS 2',
          items: [
            'module-1-ros2-nervous-system/introduction-to-ros2-for-physical-ai',
            'module-1-ros2-nervous-system/ros2-communication-model',
            'module-1-ros2-nervous-system/robot-structure-with-urdf',
            'module-1-ros2-nervous-system/module-checklist',
            'module-1-ros2-nervous-system/edge-cases',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: Digital Twin',
          items: [
            'module-2/README',
            {
              type: 'category',
              label: 'Chapter 1: Physics Simulation with Gazebo',
              items: [
                'module-2/chapter-1-physics-simulation-gazebo/index',
                'module-2/chapter-1-physics-simulation-gazebo/setup',
                'module-2/chapter-1-physics-simulation-gazebo/basic-models',
                'module-2/chapter-1-physics-simulation-gazebo/physics-principles',
                'module-2/chapter-1-physics-simulation-gazebo/gravity-tutorial',
                'module-2/chapter-1-physics-simulation-gazebo/collision-tutorial',
                {
                  type: 'category',
                  label: 'Chapter 1 Exercises',
                  items: [
                    'module-2/chapter-1-physics-simulation-gazebo/exercises/exercise-1',
                  ],
                },
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Digital Twins & HRI using Unity',
              items: [
                'module-2/chapter-2-digital-twins-unity/index',
                'module-2/chapter-2-digital-twins-unity/unity-setup',
                'module-2/chapter-2-digital-twins-unity/digital-twin-creation',
                'module-2/chapter-2-digital-twins-unity/hri-fundamentals',
                'module-2/chapter-2-digital-twins-unity/humanoid-integration',
                'module-2/chapter-2-digital-twins-unity/ui-controls',
                'module-2/chapter-2-digital-twins-unity/hri-examples',
                {
                  type: 'category',
                  label: 'Chapter 2 Exercises',
                  items: [
                    'module-2/chapter-2-digital-twins-unity/exercises/exercise-1',
                  ],
                },
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Sensor Simulation & Validation',
              items: [
                'module-2/chapter-3-sensor-simulation/index',
                'module-2/chapter-3-sensor-simulation/fundamentals',
                'module-2/chapter-3-sensor-simulation/lidar-simulation',
                'module-2/chapter-3-sensor-simulation/depth-camera-simulation',
                'module-2/chapter-3-sensor-simulation/imu-simulation',
                'module-2/chapter-3-sensor-simulation/sensor-integration',
                'module-2/chapter-3-sensor-simulation/validation',
                {
                  type: 'category',
                  label: 'Chapter 3 Exercises',
                  items: [
                    'module-2/chapter-3-sensor-simulation/exercises/exercise-1',
                  ],
                },
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Module 3: AI Robot Brain',
          items: [
            {
              type: 'category',
              label: 'Chapter 1: Isaac Sim for Photorealistic Simulation',
              items: [
                'module-3/chapter-1-isaac-sim/introduction',
                'module-3/chapter-1-isaac-sim/setup',
                'module-3/chapter-1-isaac-sim/simulation-basics',
                'module-3/chapter-1-isaac-sim/photorealistic-rendering',
                'module-3/chapter-1-isaac-sim/sensor-config',
                'module-3/chapter-1-isaac-sim/environment-model',
                'module-3/chapter-1-isaac-sim/robot-model',
                'module-3/chapter-1-isaac-sim/validation',
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Isaac ROS for VSLAM',
              items: [
                'module-3/chapter-2-isaac-ros/vslam-introduction',
                'module-3/chapter-2-isaac-ros/hardware-acceleration',
                'module-3/chapter-2-isaac-ros/visual-perception',
                'module-3/chapter-2-isaac-ros/sensor-integration',
                'module-3/chapter-2-isaac-ros/vslam-data-model',
                'module-3/chapter-2-isaac-ros/mapping-examples',
                'module-3/chapter-2-isaac-ros/performance-tips',
                'module-3/chapter-2-isaac-ros/sim-integration',
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Nav2 for Humanoid Navigation',
              items: [
                'module-3/chapter-3-nav2-humanoid/path-planning-basics',
                'module-3/chapter-3-nav2-humanoid/bipedal-navigation',
                'module-3/chapter-3-nav2-humanoid/humanoid-locomotion',
                'module-3/chapter-3-nav2-humanoid/obstacle-avoidance',
                'module-3/chapter-3-nav2-humanoid/navigation-map',
                'module-3/chapter-3-nav2-humanoid/planning-examples',
                'module-3/chapter-3-nav2-humanoid/integration',
                'module-3/chapter-3-nav2-humanoid/stability-considerations',
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action',
          items: [
            'module-4/intro',
            'module-4/voice-to-action',
            'module-4/cognitive-planning',
            'module-4/capstone-autonomous-humanoid',
            'module-4/whisper-setup-guide',
            'module-4/llm-cognitive-planning-setup-guide',
            'module-4/vla-integration-setup-guide',
            'module-4/module-4-summary',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
