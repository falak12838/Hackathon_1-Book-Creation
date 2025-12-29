# Quickstart Guide: Digital Twin Simulation with Gazebo & Unity

## Prerequisites
- Node.js 18+ for Docusaurus
- Gazebo Harmonic or Humble Hawksbill
- Unity 2022.3 LTS
- ROS 2 Humble Hawksbill
- Git for version control

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Docusaurus Dependencies
```bash
npm install
```

### 3. Verify Gazebo Installation
```bash
gazebo --version
```

### 4. Verify ROS 2 Installation
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

## Building the Documentation
```bash
npm run build
```

## Running the Documentation Locally
```bash
npm start
```

## Chapter 1: Physics Simulation with Gazebo
1. Navigate to the Gazebo examples directory
2. Launch Gazebo with the humanoid model: `ros2 launch <package> <launch_file>`
3. Follow the tutorial steps in the documentation

## Chapter 2: Digital Twins & HRI using Unity
1. Open the Unity project in the unity-examples directory
2. Load the humanoid scene
3. Follow the HRI implementation steps in the documentation

## Chapter 3: Sensor Simulation & Validation
1. Run the sensor simulation examples
2. Validate sensor outputs against expected patterns
3. Complete the validation exercises in the documentation

## Troubleshooting
- If Gazebo fails to start, ensure GPU drivers are properly installed
- If ROS 2 commands fail, verify the ROS 2 environment is sourced
- If Unity project doesn't load, check Unity version compatibility