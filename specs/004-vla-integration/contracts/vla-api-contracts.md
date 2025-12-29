# VLA System API Contracts

## Voice Command Service

### POST /voice/process
**Description**: Process incoming voice command and return action plan

**Request**:
- Content-Type: multipart/form-data or application/json
- Body:
  - audio_file (file, optional): Audio file containing voice command
  - audio_data (string, optional): Base64 encoded audio data
  - text_command (string, optional): Direct text command (for testing)

**Response**:
- 200: OK
  - Content-Type: application/json
  - Body:
    ```json
    {
      "command_id": "string",
      "transcribed_text": "string",
      "confidence_score": "float",
      "action_plan": {
        "plan_id": "string",
        "steps": [
          {
            "action_type": "string",
            "parameters": "object",
            "priority": "enum"
          }
        ],
        "estimated_duration": "float"
      }
    }
    ```
- 400: Bad Request (invalid input)
- 500: Internal Server Error (processing failed)

## Cognitive Planning Service

### POST /planning/generate
**Description**: Generate action plan from natural language command

**Request**:
- Content-Type: application/json
- Body:
  ```json
  {
    "command_text": "string",
    "environment_context": "object",
    "robot_capabilities": "object",
    "constraints": {
      "max_duration": "float",
      "safety_requirements": "object"
    }
  }
  ```

**Response**:
- 200: OK
  - Content-Type: application/json
  - Body:
    ```json
    {
      "plan_id": "string",
      "action_sequence": [
        {
          "action_id": "string",
          "action_type": "string",
          "parameters": "object",
          "preconditions": "array",
          "postconditions": "array",
          "estimated_duration": "float"
        }
      ],
      "success_probability": "float",
      "alternative_plans": "array"
    }
    ```
- 400: Bad Request (invalid command)
- 409: Conflict (command conflicts with current state)
- 500: Internal Server Error (planning failed)

## Robot Control Service

### POST /robot/action
**Description**: Execute a specific action on the humanoid robot

**Request**:
- Content-Type: application/json
- Body:
  ```json
  {
    "action_type": "enum(navigate, manipulate, speak, gesture, stop)",
    "parameters": {
      "target_position": {
        "x": "float",
        "y": "float",
        "theta": "float"
      },
      "object_id": "string",
      "message": "string",
      "joint_positions": "object"
    },
    "timeout": "float",
    "priority": "enum"
  }
  ```

**Response**:
- 200: OK
  - Content-Type: application/json
  - Body:
    ```json
    {
      "action_id": "string",
      "status": "enum(accepted, rejected, executing)",
      "estimated_completion": "float"
    }
    ```
- 202: Accepted (action queued)
- 400: Bad Request (invalid action)
- 409: Conflict (action conflicts with current state)
- 423: Locked (robot busy with higher priority task)

### GET /robot/state
**Description**: Get current state of the humanoid robot

**Response**:
- 200: OK
  - Content-Type: application/json
  - Body:
    ```json
    {
      "robot_id": "string",
      "position": {
        "x": "float",
        "y": "float",
        "theta": "float"
      },
      "joint_positions": "object",
      "battery_level": "float",
      "operational_status": "enum",
      "last_updated": "string"
    }
    ```

## Safety Service

### POST /safety/emergency-stop
**Description**: Trigger emergency stop for all robot operations

**Request**:
- Content-Type: application/json
- Body:
  ```json
  {
    "reason": "string",
    "initiator": "string"
  }
  ```

**Response**:
- 200: OK
  - Content-Type: application/json
  - Body:
    ```json
    {
      "status": "stopped",
      "actions_terminated": "array",
      "timestamp": "string"
    }
    ```

### POST /safety/resume
**Description**: Resume normal operations after emergency stop

**Response**:
- 200: OK
  - Content-Type: application/json
  - Body:
    ```json
    {
      "status": "resumed",
      "timestamp": "string"
    }
    ```