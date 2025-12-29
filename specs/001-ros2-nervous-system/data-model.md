# Data Model: Robotic Nervous System (ROS 2) Educational Content

## Content Structure Entities

### Module
- **Name**: String (required) - The module title (e.g., "Module 1 - The Robotic Nervous System")
- **Description**: String (required) - Brief overview of the module's purpose and content
- **Target Audience**: Array of strings - List of intended reader types (e.g., ["AI students", "robotics developers"])
- **Prerequisites**: Array of strings - Knowledge required before starting the module
- **Learning Objectives**: Array of strings - What learners will be able to do after completing the module
- **Chapters**: Array of Chapter entities - The chapters contained in this module

### Chapter
- **Title**: String (required) - The chapter title
- **Subtitle**: String (optional) - Additional descriptive text for the chapter
- **Content Path**: String (required) - File path to the chapter's Markdown content
- **Order**: Integer (required) - The position of this chapter within the module
- **Learning Goals**: Array of strings - Specific skills or knowledge to be gained
- **Prerequisites**: Array of strings - Knowledge needed for this specific chapter
- **Examples**: Array of Example entities - Practical examples included in the chapter

### Example
- **Title**: String (required) - Brief title of the example
- **Type**: String (required) - Category of example (e.g., "code", "diagram", "simulation")
- **Description**: String (required) - Explanation of what the example demonstrates
- **File Path**: String (required) - Path to the example file or code
- **Dependencies**: Array of strings - ROS 2 packages or tools required for the example

### ROS 2 Concept
- **Name**: String (required) - The name of the ROS 2 concept (e.g., "Node", "Topic", "Service")
- **Definition**: String (required) - Clear explanation of the concept
- **Usage**: String (required) - How the concept is typically used in ROS 2 systems
- **Examples**: Array of Example entities - Examples demonstrating the concept
- **Related Concepts**: Array of strings - Other ROS 2 concepts that relate to this one

### URDF Model
- **Name**: String (required) - Name of the robot model
- **Description**: String (required) - Brief description of the robot
- **Links**: Array of Link entities - Physical links in the robot structure
- **Joints**: Array of Joint entities - Joints connecting the links
- **File Path**: String (required) - Path to the URDF file
- **Validation Status**: String - Current status of URDF validation (e.g., "valid", "invalid", "pending")

### Link
- **Name**: String (required) - Name of the link
- **Visual**: Object - Visual properties of the link (geometry, material, etc.)
- **Collision**: Object - Collision properties of the link
- **Inertial**: Object - Mass, center of mass, and inertia properties

### Joint
- **Name**: String (required) - Name of the joint
- **Type**: String (required) - Type of joint (e.g., "revolute", "prismatic", "fixed")
- **Parent**: String (required) - Name of the parent link
- **Child**: String (required) - Name of the child link
- **Limits**: Object - Joint limits (for movable joints)

## Validation Rules

### Module Validation
- Module must have at least one chapter
- Module title must be unique within the book
- All learning objectives must be achievable through the contained chapters

### Chapter Validation
- Chapter title must be unique within the module
- Chapter order must be sequential (1, 2, 3, etc.)
- All referenced examples must exist
- Chapter must include at least one learning goal

### Example Validation
- Example file path must exist and be accessible
- Example dependencies must be documented
- Example must be tested and verified as functional

### URDF Model Validation
- URDF file must be syntactically correct XML
- All referenced mesh files must exist
- Joint limits must be within reasonable ranges
- Model must be kinematically valid

## State Transitions

### Content Development States
- **Draft**: Initial content creation stage
- **Reviewed**: Content reviewed by subject matter expert
- **Validated**: Technical examples verified as functional
- **Published**: Content ready for learner consumption

### Validation Process
- Content moves from Draft → Reviewed → Validated → Published
- Content may revert to Draft if issues are found during review or validation
- Each state transition requires specific criteria to be met