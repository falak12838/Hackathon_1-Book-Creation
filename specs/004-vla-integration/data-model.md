# Data Model: Vision-Language-Action (VLA) Integration

## Voice Command Entity
- **name**: VoiceCommand
- **fields**:
  - command_text (string): The transcribed voice command text
  - confidence_score (float): Confidence level of speech recognition (0.0-1.0)
  - timestamp (datetime): When the command was received
  - user_id (string): Optional identifier for multi-user systems
  - processed_status (enum): Status of command processing (pending, processing, completed, failed)
- **relationships**:
  - One-to-many with ActionPlan (one command may generate multiple action plans)
- **validation rules**:
  - command_text must not be empty
  - confidence_score must be between 0.0 and 1.0
  - timestamp must be current or past

## Action Plan Entity
- **name**: ActionPlan
- **fields**:
  - plan_id (string): Unique identifier for the action plan
  - steps (list): Sequence of ROS 2 actions to execute
  - priority (enum): Priority level (low, medium, high, emergency)
  - status (enum): Execution status (pending, executing, completed, failed, interrupted)
  - created_timestamp (datetime): When the plan was created
  - completed_timestamp (datetime): When the plan was completed (nullable)
- **relationships**:
  - Many-to-one with VoiceCommand (many action plans may come from one command)
  - One-to-many with RobotState (tracks state changes during execution)
- **validation rules**:
  - steps must contain at least one action
  - status transitions must follow valid sequence

## Robot State Entity
- **name**: RobotState
- **fields**:
  - robot_id (string): Unique identifier for the robot
  - position_x (float): X coordinate in environment
  - position_y (float): Y coordinate in environment
  - orientation (float): Robot orientation in radians
  - joint_positions (dict): Current positions of all joints
  - battery_level (float): Battery level as percentage (0.0-1.0)
  - operational_status (enum): Current operational status (idle, moving, manipulating, error)
  - last_updated (datetime): When state was last updated
- **relationships**:
  - Many-to-many with ActionPlan (multiple plans may reference robot state)
- **validation rules**:
  - position values must be within environment bounds
  - battery_level must be between 0.0 and 1.0

## Environmental Context Entity
- **name**: EnvironmentalContext
- **fields**:
  - context_id (string): Unique identifier for the context snapshot
  - objects (list): List of recognized objects in the environment
  - obstacles (list): List of detected obstacles
  - navigable_areas (list): List of accessible areas
  - timestamp (datetime): When the context was captured
  - confidence_map (dict): Confidence scores for object recognition
- **relationships**:
  - One-to-many with ActionPlan (context used for planning)
- **validation rules**:
  - objects list must contain valid object definitions
  - timestamp must be recent (less than 5 seconds old for validity)

## State Transitions

### ActionPlan State Transitions:
- pending → executing (when execution starts)
- executing → completed (when all steps finish successfully)
- executing → failed (when an action fails)
- executing → interrupted (when emergency stop is triggered)
- pending → failed (when validation fails)

### RobotState Operational Status Transitions:
- idle → moving (when navigation command received)
- idle → manipulating (when manipulation command received)
- moving → idle (when navigation completes)
- manipulating → idle (when manipulation completes)
- [any] → error (when safety system activates)