# Quickstart Guide: Vision-Language-Action (VLA) Integration

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Python 3.11 or higher
- OpenAI API key for Whisper service
- A humanoid robot with ROS 2 support (or Gazebo simulation)
- Docusaurus development environment

## Setup Instructions

### 1. Environment Setup

```bash
# Clone the repository
git clone [repository-url]
cd [repository-name]

# Create a virtual environment
python -m venv vla_env
source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate

# Install ROS 2 dependencies
pip install rclpy std_msgs geometry_msgs sensor_msgs

# Install OpenAI dependencies
pip install openai python-dotenv

# Install Docusaurus dependencies
cd frontend-book
npm install
```

### 2. Configuration

Create a `.env` file in the root directory:

```env
OPENAI_API_KEY=your_openai_api_key_here
ROS_DOMAIN_ID=42
WHISPER_MODEL=whisper-1
```

### 3. Launch the VLA System

```bash
# Terminal 1: Start ROS 2 environment
source /opt/ros/humble/setup.bash
cd [workspace_dir]
source install/setup.bash

# Terminal 2: Launch the voice command node
ros2 run vla voice_command_node.py

# Terminal 3: Launch the cognitive planner node
ros2 run vla cognitive_planner_node.py

# Terminal 4: Launch the humanoid controller node
ros2 run vla humanoid_controller_node.py
```

### 4. Run the Docusaurus Documentation Site

```bash
cd frontend-book
npm start
```

The documentation will be available at http://localhost:3000

## Basic Usage

### Voice Command Test

1. Ensure all nodes are running
2. Speak a simple command like "Move forward 1 meter"
3. Observe the robot executing the movement

### Cognitive Planning Test

1. Provide a multi-step command like "Go to the kitchen and bring me the cup"
2. Watch as the cognitive planner generates a sequence of actions
3. Monitor the robot executing the planned sequence

### Safety Override

1. At any time, say "Emergency stop" or use the keyboard interrupt
2. The safety supervisor will immediately halt all robot operations
3. Resume operations by issuing a "Resume" command

## Running Tests

```bash
# Run Python unit tests
python -m pytest tests/unit/

# Run integration tests
python -m pytest tests/integration/

# Run Docusaurus documentation tests
cd frontend-book
npm test
```

## Troubleshooting

### Common Issues

- **Whisper API errors**: Verify your OpenAI API key is correct and has sufficient credits
- **ROS 2 connection issues**: Ensure all nodes are on the same ROS_DOMAIN_ID
- **Robot not responding**: Check robot power and ROS 2 communication
- **Speech recognition poor quality**: Ensure quiet environment and proper microphone

### Performance Tips

- For best voice recognition, use in a quiet environment
- Position microphone within 1 meter of speaker
- Ensure humanoid robot has sufficient battery power (>20%)
- Monitor system resources during complex planning operations