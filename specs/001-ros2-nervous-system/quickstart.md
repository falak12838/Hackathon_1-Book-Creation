# Quickstart Guide: ROS 2 Educational Book Development

## Prerequisites

- Node.js 18+ and npm/yarn
- Git
- A GitHub account
- Basic knowledge of Markdown and Docusaurus

## Setup Development Environment

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
cd frontend-book
npm install
```

### 3. Initialize Docusaurus
```bash
npm run start
```

This will start the development server and open the documentation site in your browser.

## Creating Module 1 Content

### 1. Create the Module Directory
```bash
mkdir docs/module-1-ros2-nervous-system
```

### 2. Create Chapter Files

#### Chapter 1: Introduction to ROS 2 for Physical AI
```bash
touch docs/module-1-ros2-nervous-system/introduction-to-ros2-for-physical-ai.md
```

Add the following content to the file:
```markdown
---
title: Introduction to ROS 2 for Physical AI
sidebar_position: 1
---

# Introduction to ROS 2 for Physical AI

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## Why ROS 2 Matters for Humanoids

Humanoid robots require sophisticated coordination between multiple systems - perception, planning, control, and execution. ROS 2 provides the middleware that allows these systems to communicate effectively, making it an ideal choice for humanoid robotics development.

## OOS Concepts (Open, Operate, Share)

The OOS philosophy in ROS 2 emphasizes:
- Open: Accessible and transparent development
- Operate: Reliable and robust operation
- Share: Collaboration and knowledge sharing

[Content continues with more details...]
```

#### Chapter 2: ROS 2 Communication Model
```bash
touch docs/module-1-ros2-nervous-system/ros2-communication-model.md
```

#### Chapter 3: Robot Structure with URDF
```bash
touch docs/module-1-ros2-nervous-system/robot-structure-with-urdf.md
```

### 3. Configure Sidebar

Update the `sidebar.js` file to include your new module and chapters:

```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System',
      items: [
        'module-1-ros2-nervous-system/introduction-to-ros2-for-physical-ai',
        'module-1-ros2-nervous-system/ros2-communication-model',
        'module-1-ros2-nervous-system/robot-structure-with-urdf',
      ],
    },
    // ... other modules
  ],
};
```

### 4. Build and Test

```bash
npm run build
```

This will generate the static site in the `build/` directory.

## ROS 2 Development Environment

For the practical examples in the book, you'll also need:

### Install ROS 2
Follow the official installation guide for your OS:
- Ubuntu: [ROS 2 Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Windows: [ROS 2 Installation](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
- macOS: [ROS 2 Installation](https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html)

### Verify Installation
```bash
source /opt/ros/humble/setup.bash  # For Ubuntu with Humble
ros2 --version
```

### Create a Workspace for Examples
```bash
mkdir -p ~/ros2_book_examples/src
cd ~/ros2_book_examples
colcon build
source install/setup.bash
```

## Running Examples

Each chapter will include practical examples. To run them:

1. Navigate to the example directory
2. Source your ROS 2 environment
3. Run the example as described in the chapter

Example:
```bash
cd ~/ros2_book_examples/src/my_first_package
source ~/ros2_book_examples/install/setup.bash
python3 my_first_node.py
```

## Validation

### Content Validation
- Ensure all links are working
- Verify all code examples run correctly
- Confirm URDF files are valid

### URDF Validation
```bash
check_urdf /path/to/your/robot.urdf
```

## Deployment

The site is deployed using GitHub Pages:

1. Commit your changes
2. Push to the main branch
3. GitHub Actions will automatically build and deploy the site