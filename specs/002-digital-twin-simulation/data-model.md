# Data Model: Digital Twin Simulation with Gazebo & Unity

## Entities

### Simulation Environment
- **Name**: Unique identifier for the simulation environment
- **Description**: Text description of the environment
- **Physics Properties**: Gravity, friction, damping coefficients
- **Objects**: Collection of physical objects in the environment
- **Lighting**: Lighting configuration for visualization
- **Validation Rules**: Environment must have valid physics properties

### Digital Twin
- **Name**: Unique identifier for the digital twin
- **Model Type**: Type of humanoid robot model (URDF, FBX, etc.)
- **Properties**: Physical properties matching real robot (mass, dimensions, etc.)
- **State**: Current state of the twin (position, orientation, joint angles)
- **Sensors**: Collection of simulated sensors attached to the twin
- **Validation Rules**: Must have valid URDF/3D model file, valid physical properties

### Sensor Data
- **Type**: Sensor type (LiDAR, depth camera, IMU, etc.)
- **Timestamp**: Time of sensor reading
- **Values**: Sensor output values (point cloud, image, acceleration, etc.)
- **Accuracy**: Accuracy rating of the sensor data
- **Source**: Reference to the sensor that generated the data
- **Validation Rules**: Must have valid timestamp and values within expected ranges

### Chapter Content
- **Title**: Chapter title
- **Content**: Markdown content for the chapter
- **Assets**: Associated files (images, models, configurations)
- **Examples**: Code/configuration examples included in chapter
- **Validation Rules**: Must have valid markdown syntax, all referenced assets must exist

### Student Exercise
- **Title**: Exercise title
- **Description**: Description of the exercise
- **Prerequisites**: Skills/knowledge required
- **Steps**: Sequential steps for completing exercise
- **Expected Outcome**: What student should achieve
- **Validation Rules**: Must have complete steps and clear expected outcome