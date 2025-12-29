# Vision-Language-Action (VLA) Integration Development Guidelines

Auto-generated from feature plan. Last updated: 2025-12-29

## Active Technologies

- **Docusaurus**: Static site generator for documentation (v3.0+)
- **OpenAI Whisper**: Speech recognition for voice command processing (Large v3)
- **OpenAI GPT**: Large language models for cognitive planning and language processing
- **ROS 2**: Middleware for communication between components (Humble Hawksbill or newer)
- **Python**: Primary language for Whisper and LLM integration (rclpy for ROS 2)
- **C++**: ROS 2 client library (rclcpp) for performance-critical components
- **Markdown (.md)**: Content format for all documentation
- **Node.js**: Runtime environment for Docusaurus (v18+)

## Project Structure

```text
vla-integration/
├── frontend-book/          # Docusaurus documentation
│   ├── docs/
│   │   ├── module-4/       # VLA content
│   │   │   ├── voice-to-action.md           # Voice-to-Action chapter
│   │   │   ├── cognitive-planning.md        # Cognitive Planning chapter
│   │   │   └── capstone-autonomous-humanoid.md # Capstone chapter
│   │   ├── static/         # Static assets
│   │   └── ...
│   ├── docusaurus.config.js # Site configuration
│   ├── package.json        # Dependencies
│   └── ...
├── vla_workspace/          # VLA workspace for examples
│   ├── src/
│   │   ├── vla_examples/   # VLA integration examples
│   │   └── ...
└── specs/                  # Feature specifications
    └── 004-vla-integration/
        ├── spec.md
        ├── plan.md
        ├── research.md
        ├── data-model.md
        ├── quickstart.md
        ├── contracts/
        └── checklists/
```

## Commands

### Docusaurus Commands:
```bash
# Install dependencies
npm install

# Start development server
npm start

# Build static site
npm run build

# Deploy to GitHub Pages
npm run deploy
```

### VLA Integration Commands:
```bash
# Create and build VLA workspace
mkdir -p ~/vla_ws/src && cd ~/vla_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source VLA workspace
source install/setup.bash

# Run voice-to-action example
python3 voice_to_action_demo.py

# Run cognitive planning example
python3 cognitive_planning_demo.py

# Run complete VLA integration
python3 complete_vla_demo.py
```

### OpenAI Whisper Commands:
- Process audio files with Whisper: `whisper audio_file.wav --model large`
- Use Whisper API for real-time voice processing
- Configure Whisper for robotics-specific audio environments

## Code Style

### Python (VLA Integration):
- Follow PEP 8 style guide
- Use type hints for function parameters and return values
- Implement proper error handling with try-catch blocks
- Use logging instead of print statements for debugging
- Follow safety patterns for robot command validation

### ROS 2 Python (rclpy):
- Follow ROS 2 Python style guide
- Use proper node lifecycle management
- Implement action servers for long-running tasks
- Include proper error handling and logging for robot operations

### Docusaurus Markdown:
- Use proper heading hierarchy (##, ###, etc.)
- Include code blocks with appropriate language tags
- Use Docusaurus-specific components when needed (e.g., `<Tabs>`, `<TabItem>`)
- Include embedded examples and interactive elements for VLA tutorials

## Recent Changes

- **VLA Integration Module**: Created comprehensive educational content covering OpenAI Whisper integration, cognitive planning with LLMs, and complete VLA system integration for humanoid robots
- **Docusaurus Structure**: Created module-4 documentation with three chapters: Voice-to-Action, Cognitive Planning, and Capstone Autonomous Humanoid
- **VLA Ecosystem Integration**: Implemented tutorials for voice command processing, language-to-action translation, and autonomous task execution

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->