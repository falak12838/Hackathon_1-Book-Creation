# Vision-Language-Action (VLA) System for Humanoid Robotics

This project implements a Vision-Language-Action (VLA) system that enables natural voice interaction with humanoid robots. The system integrates voice processing, cognitive planning, and robot control to create an autonomous humanoid capable of understanding and executing complex natural language commands.

## Features

- **Voice Command Processing**: Uses OpenAI Whisper for accurate voice-to-text conversion
- **Cognitive Planning**: Translates natural language commands into sequences of robot actions
- **Autonomous Execution**: Executes complex multi-step tasks on humanoid robots
- **Safety Supervisor**: Includes emergency stop and safety monitoring capabilities
- **Human-in-the-Loop**: Maintains human oversight and intervention capabilities

## Architecture

The VLA system consists of three main modules:

1. **Voice-to-Action**: Converts voice commands to robot actions
2. **Cognitive Planning**: Translates language to ROS 2 action sequences
3. **Autonomous Humanoid**: Complete integration and task execution

## Installation

### Prerequisites

- ROS 2 Humble Hawksbill
- Python 3.8+
- OpenAI API key

### Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd vla-system
   ```

2. Create a virtual environment:
   ```bash
   python -m venv vla_env
   source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate
   ```

3. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up your OpenAI API key:
   ```bash
   cp .env.example .env
   # Edit .env and add your OpenAI API key
   ```

## Usage

### Running the System

The system consists of several ROS 2 nodes that work together:

1. **Voice Command Node**: Processes voice input
   ```bash
   ros2 run vla voice_command_node.py
   ```

2. **Cognitive Planner Node**: Plans action sequences
   ```bash
   ros2 run vla cognitive_planner_node.py
   ```

3. **Humanoid Controller Node**: Executes actions
   ```bash
   ros2 run vla humanoid_controller_node.py
   ```

### Example Commands

Once the system is running, you can issue voice commands such as:
- "Move forward 1 meter"
- "Go to the kitchen and bring me the red cup"
- "Turn left and navigate to the office"

## Documentation

Complete documentation is available in the `docs/module-4/` directory:

- [Voice-to-Action Guide](docs/module-4/voice-to-action.md)
- [Cognitive Planning Guide](docs/module-4/cognitive-planning.md)
- [Capstone Autonomous Humanoid Guide](docs/module-4/capstone-autonomous-humanoid.md)

## Project Structure

```
vla-system/
├── docs/                       # Documentation files
│   └── module-4/              # VLA module documentation
├── src/                       # Source code
│   ├── ros2_nodes/           # ROS 2 nodes
│   └── vla/                 # VLA core modules
├── requirements.txt           # Python dependencies
├── setup.py                   # Package setup
├── .env.example              # Environment variables template
└── README.md                 # This file
```

## Safety Considerations

- Always maintain human oversight during robot operation
- The system includes emergency stop capabilities
- Ensure the robot operates in a safe environment
- Monitor system performance and resource usage

## Contributing

Contributions are welcome! Please read our contributing guidelines before submitting a pull request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- OpenAI for the Whisper API
- ROS 2 community for the robotics framework
- The humanoid robotics research community