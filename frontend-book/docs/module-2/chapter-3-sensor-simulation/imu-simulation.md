# IMU Simulation Tutorial

## Overview

This tutorial covers the implementation of IMU (Inertial Measurement Unit) sensors in digital twin environments using both Gazebo and Unity. You'll learn to create realistic IMU simulation that generates accurate acceleration and orientation data for robotics applications.

## Prerequisites

- Understanding of sensor simulation fundamentals
- Basic knowledge of Gazebo and Unity
- Completion of previous chapters
- Understanding of physics principles and coordinate systems

## IMU Sensor Theory

### How IMU Sensors Work

An IMU typically combines multiple sensors to measure linear acceleration, angular velocity, and sometimes magnetic field strength:

- **Accelerometer**: Measures linear acceleration in 3D space (x, y, z)
- **Gyroscope**: Measures angular velocity (rotation rate) around 3 axes
- **Magnetometer**: Measures magnetic field strength and direction (optional)

**Key Parameters:**
- **Accelerometer Range**: ±2g to ±16g (where g is gravitational acceleration)
- **Gyroscope Range**: ±250°/s to ±2000°/s (degrees per second)
- **Magnetometer Range**: ±4800 µT (microtesla)
- **Update Rate**: 100-1000 Hz
- **Noise Density**: Specified in units per square root of Hz
- **Bias Stability**: Drift over time (e.g., °/h for gyros, µg for accelerometers)

### Coordinate Systems

IMU sensors typically use the right-hand coordinate system:
- **X-axis**: Forward direction of the robot
- **Y-axis**: Lateral direction (left is positive)
- **Z-axis**: Vertical direction (up is positive)

## Implementing IMU in Gazebo

### Step 1: Adding IMU to URDF Model

First, add an IMU sensor to your robot model in URDF:

```xml
<!-- Add to your robot URDF -->
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<!-- IMU sensor definition -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.002</bias_mean>
            <bias_stddev>0.0003</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.002</bias_mean>
            <bias_stddev>0.0003</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.002</bias_mean>
            <bias_stddev>0.0003</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <body_name>imu_link</body_name>
      <update_rate>200</update_rate>
      <gaussian_noise>1.7e-2</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

### Step 2: Configuring Advanced IMU Parameters

For more realistic IMU simulation, configure additional parameters:

```xml
<gazebo reference="imu_link">
  <sensor name="advanced_imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>500</update_rate>
    <imu>
      <!-- Angular velocity configuration -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.2e-4</stddev>
            <bias_mean>0.001</bias_mean>
            <bias_stddev>0.0002</bias_stddev>
            <dynamic_bias_stddev>0.0001</dynamic_bias_stddev>
            <dynamic_bias_correlation_time>100</dynamic_bias_correlation_time>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.2e-4</stddev>
            <bias_mean>0.001</bias_mean>
            <bias_stddev>0.0002</bias_stddev>
            <dynamic_bias_stddev>0.0001</dynamic_bias_stddev>
            <dynamic_bias_correlation_time>100</dynamic_bias_correlation_time>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.2e-4</stddev>
            <bias_mean>0.001</bias_mean>
            <bias_stddev>0.0002</bias_stddev>
            <dynamic_bias_stddev>0.0001</dynamic_bias_stddev>
            <dynamic_bias_correlation_time>100</dynamic_bias_correlation_time>
          </noise>
        </z>
      </angular_velocity>

      <!-- Linear acceleration configuration -->
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.05</bias_mean>
            <bias_stddev>0.001</bias_stddev>
            <dynamic_bias_stddev>0.0001</dynamic_bias_stddev>
            <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.05</bias_mean>
            <bias_stddev>0.001</bias_stddev>
            <dynamic_bias_stddev>0.0001</dynamic_bias_stddev>
            <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.05</bias_mean>
            <bias_stddev>0.001</bias_stddev>
            <dynamic_bias_stddev>0.0001</dynamic_bias_stddev>
            <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="advanced_imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <body_name>imu_link</body_name>
      <update_rate>500</update_rate>
      <gaussian_noise>0.017</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

### Step 3: Creating an IMU Processing Node

Create a ROS 2 node to process IMU data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Create subscriber for raw IMU data
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publisher for processed IMU data
        self.imu_publisher = self.create_publisher(Imu, '/imu/processed', 10)

        # Create publisher for orientation data
        self.orientation_publisher = self.create_publisher(Vector3, '/imu/orientation', 10)

        # IMU parameters
        self.linear_acceleration_variance = 0.017**2
        self.angular_velocity_variance = 1.2e-4**2

        # Initialize state variables
        self.orientation = R.from_quat([0, 0, 0, 1])  # Identity quaternion
        self.last_time = None

        self.get_logger().info('IMU Processor node initialized')

    def imu_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Calculate time difference
        dt = 0
        if self.last_time is not None:
            dt = current_time - self.last_time

        self.last_time = current_time

        # Process the IMU data
        processed_imu = self.process_imu_data(msg, dt)

        # Publish processed data
        self.imu_publisher.publish(processed_imu)

        # Extract and publish orientation
        orientation_msg = Vector3()
        euler = self.orientation.as_euler('xyz', degrees=True)
        orientation_msg.x = euler[0]  # Roll
        orientation_msg.y = euler[1]  # Pitch
        orientation_msg.z = euler[2]  # Yaw
        self.orientation_publisher.publish(orientation_msg)

    def process_imu_data(self, raw_imu, dt):
        # Create a new IMU message for processed data
        processed_imu = Imu()
        processed_imu.header = raw_imu.header

        # Apply basic filtering to the linear acceleration
        processed_imu.linear_acceleration.x = self.filter_acceleration(
            raw_imu.linear_acceleration.x
        )
        processed_imu.linear_acceleration.y = self.filter_acceleration(
            raw_imu.linear_acceleration.y
        )
        processed_imu.linear_acceleration.z = self.filter_acceleration(
            raw_imu.linear_acceleration.z
        )

        # Apply basic filtering to the angular velocity
        processed_imu.angular_velocity.x = self.filter_angular_velocity(
            raw_imu.angular_velocity.x
        )
        processed_imu.angular_velocity.y = self.filter_angular_velocity(
            raw_imu.angular_velocity.y
        )
        processed_imu.angular_velocity.z = self.filter_angular_velocity(
            raw_imu.angular_velocity.z
        )

        # Update orientation using angular velocity integration
        if dt > 0:
            self.update_orientation(
                raw_imu.angular_velocity.x,
                raw_imu.angular_velocity.y,
                raw_imu.angular_velocity.z,
                dt
            )

        # Set the orientation quaternion
        quat = self.orientation.as_quat()
        processed_imu.orientation.x = quat[0]
        processed_imu.orientation.y = quat[1]
        processed_imu.orientation.z = quat[2]
        processed_imu.orientation.w = quat[3]

        # Set covariance matrices
        self.set_covariance_matrices(processed_imu)

        return processed_imu

    def filter_acceleration(self, value):
        # Simple filtering - in practice, you'd use more sophisticated filters
        # like Kalman filters or complementary filters
        return value

    def filter_angular_velocity(self, value):
        # Simple filtering for angular velocity
        return value

    def update_orientation(self, wx, wy, wz, dt):
        # Integrate angular velocity to update orientation
        # Convert angular velocity to quaternion derivative
        omega = np.array([wx, wy, wz])
        current_quat = self.orientation.as_quat()

        # Calculate quaternion derivative
        # q_dot = 0.5 * q * omega_quat
        omega_quat = np.array([0, wx, wy, wz])
        quat_dot = self.quaternion_multiply(current_quat, omega_quat) * 0.5

        # Update quaternion
        new_quat = current_quat + quat_dot * dt

        # Normalize quaternion
        new_quat = new_quat / np.linalg.norm(new_quat)

        # Update orientation
        self.orientation = R.from_quat(new_quat)

    def quaternion_multiply(self, q1, q2):
        # Multiply two quaternions
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    def set_covariance_matrices(self, imu_msg):
        # Set the covariance matrices for the IMU data
        # These values should be based on the sensor specifications
        linear_acceleration_covariance = [0.0] * 9
        linear_acceleration_covariance[0] = self.linear_acceleration_variance  # x
        linear_acceleration_covariance[4] = self.linear_acceleration_variance  # y
        linear_acceleration_covariance[8] = self.linear_acceleration_variance  # z
        imu_msg.linear_acceleration_covariance = linear_acceleration_covariance

        angular_velocity_covariance = [0.0] * 9
        angular_velocity_covariance[0] = self.angular_velocity_variance  # x
        angular_velocity_covariance[4] = self.angular_velocity_variance  # y
        angular_velocity_covariance[8] = self.angular_velocity_variance  # z
        imu_msg.angular_velocity_covariance = angular_velocity_covariance

        # Orientation covariance - set to 0 if not calculated
        orientation_covariance = [0.0] * 9
        imu_msg.orientation_covariance = orientation_covariance

def main(args=None):
    rclpy.init(args=args)
    imu_processor = IMUProcessor()

    try:
        rclpy.spin(imu_processor)
    except KeyboardInterrupt:
        pass
    finally:
        imu_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing IMU in Unity

### Step 1: Creating an IMU Simulation Script

Create a C# script to simulate IMU functionality in Unity:

```csharp
using UnityEngine;
using System.Collections.Generic;
using System;

public class UnityIMU : MonoBehaviour
{
    [Header("IMU Configuration")]
    [SerializeField] private float updateRate = 200f; // Hz
    [SerializeField] private float accelerometerRange = 16f; // ±16g
    [SerializeField] private float gyroscopeRange = 2000f; // ±2000°/s
    [SerializeField] private float magnetometerRange = 4800f; // ±4800 µT

    [Header("Noise Parameters")]
    [SerializeField] private float accelerometerNoiseStdDev = 0.017f;
    [SerializeField] private float gyroscopeNoiseStdDev = 0.00012f;
    [SerializeField] private float magnetometerNoiseStdDev = 0.1f;

    [Header("Bias Parameters")]
    [SerializeField] private float accelerometerBiasStdDev = 0.001f;
    [SerializeField] private float gyroscopeBiasStdDev = 0.0002f;
    [SerializeField] private float magnetometerBiasStdDev = 0.01f;

    [Header("Output")]
    [SerializeField] private bool publishData = true;

    // IMU data
    private Vector3 linearAcceleration;
    private Vector3 angularVelocity;
    private Vector3 magneticField;
    private Quaternion orientation;
    private float nextUpdate = 0f;

    // Noise and bias tracking
    private Vector3 accelerometerBias;
    private Vector3 gyroscopeBias;
    private Vector3 magnetometerBias;

    // Previous state for integration
    private Vector3 previousVelocity;
    private Quaternion previousOrientation;

    void Start()
    {
        InitializeIMU();
    }

    void InitializeIMU()
    {
        // Initialize biases
        accelerometerBias = new Vector3(
            UnityEngine.Random.Range(-accelerometerBiasStdDev, accelerometerBiasStdDev),
            UnityEngine.Random.Range(-accelerometerBiasStdDev, accelerometerBiasStdDev),
            UnityEngine.Random.Range(-accelerometerBiasStdDev, accelerometerBiasStdDev)
        );

        gyroscopeBias = new Vector3(
            UnityEngine.Random.Range(-gyroscopeBiasStdDev, gyroscopeBiasStdDev),
            UnityEngine.Random.Range(-gyroscopeBiasStdDev, gyroscopeBiasStdDev),
            UnityEngine.Random.Range(-gyroscopeBiasStdDev, gyroscopeBiasStdDev)
        );

        magnetometerBias = new Vector3(
            UnityEngine.Random.Range(-magnetometerBiasStdDev, magnetometerBiasStdDev),
            UnityEngine.Random.Range(-magnetometerBiasStdDev, magnetometerBiasStdDev),
            UnityEngine.Random.Range(-magnetometerBiasStdDev, magnetometerBiasStdDev)
        );

        // Initialize orientation
        orientation = transform.rotation;
        previousOrientation = orientation;

        // Initialize velocity
        previousVelocity = Vector3.zero;
    }

    void Update()
    {
        if (Time.time >= nextUpdate)
        {
            SimulateIMU();
            nextUpdate = Time.time + (1f / updateRate);
        }
    }

    void SimulateIMU()
    {
        // Get the true values from Unity's physics system
        Vector3 trueLinearAcceleration = GetTrueLinearAcceleration();
        Vector3 trueAngularVelocity = GetTrueAngularVelocity();
        Vector3 trueMagneticField = GetTrueMagneticField();

        // Add noise and bias to the measurements
        linearAcceleration = AddNoiseAndBias(trueLinearAcceleration, accelerometerNoiseStdDev, accelerometerBias);
        angularVelocity = AddNoiseAndBias(trueAngularVelocity, gyroscopeNoiseStdDev, gyroscopeBias);
        magneticField = AddNoiseAndBias(trueMagneticField, magnetometerNoiseStdDev, magnetometerBias);

        // Update orientation by integrating angular velocity
        UpdateOrientation();

        // Publish data if enabled
        if (publishData)
        {
            PublishIMUData();
        }
    }

    Vector3 GetTrueLinearAcceleration()
    {
        // Get the true linear acceleration from the object's movement
        // This could be from physics, animation, or other movement systems
        Rigidbody rb = GetComponent<Rigidbody>();

        if (rb != null)
        {
            // Get acceleration from Rigidbody
            return rb.velocity - previousVelocity;
        }
        else
        {
            // If no Rigidbody, calculate from position changes
            // This is a simplified approach
            Vector3 currentVelocity = (transform.position - transform.position) / Time.deltaTime; // This would be from previous frame
            Vector3 acceleration = (currentVelocity - previousVelocity) / Time.deltaTime;
            previousVelocity = currentVelocity;
            return acceleration;
        }
    }

    Vector3 GetTrueAngularVelocity()
    {
        // Get the true angular velocity from the object's rotation
        Rigidbody rb = GetComponent<Rigidbody>();

        if (rb != null)
        {
            return rb.angularVelocity;
        }
        else
        {
            // Calculate from rotation changes
            Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(previousOrientation);
            Vector3 angularVelocity = new Vector3(
                Mathf.Rad2Deg * deltaRotation.eulerAngles.x / Time.deltaTime,
                Mathf.Rad2Deg * deltaRotation.eulerAngles.y / Time.deltaTime,
                Mathf.Rad2Deg * deltaRotation.eulerAngles.z / Time.deltaTime
            );
            return angularVelocity;
        }
    }

    Vector3 GetTrueMagneticField()
    {
        // Simulate Earth's magnetic field (approximately 25-65 µT)
        // This is a simplified model - in reality, it varies by location
        Vector3 magneticNorth = Vector3.forward * 25f; // µT
        Vector3 localMagneticField = transform.InverseTransformDirection(magneticNorth);
        return localMagneticField;
    }

    Vector3 AddNoiseAndBias(Vector3 value, float noiseStdDev, Vector3 bias)
    {
        Vector3 noise = new Vector3(
            GetGaussianNoise(noiseStdDev),
            GetGaussianNoise(noiseStdDev),
            GetGaussianNoise(noiseStdDev)
        );

        return value + noise + bias;
    }

    float GetGaussianNoise(float stdDev)
    {
        // Generate Gaussian noise using Box-Muller transform
        float u1 = UnityEngine.Random.value;
        float u2 = UnityEngine.Random.value;
        if (u1 < float.Epsilon) u1 = 0.01f; // Avoid log(0)
        float normal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
        return normal * stdDev;
    }

    void UpdateOrientation()
    {
        // Integrate angular velocity to update orientation
        float dt = 1f / updateRate; // Time step based on update rate

        // Convert angular velocity from degrees/s to radians/s
        Vector3 angularVelocityRad = angularVelocity * Mathf.Deg2Rad;

        // Calculate quaternion derivative
        Quaternion currentQuat = orientation;
        Vector4 omega = new Vector4(angularVelocityRad.x, angularVelocityRad.y, angularVelocityRad.z, 0);
        Vector4 currentQuatVec = new Vector4(currentQuat.x, currentQuat.y, currentQuat.z, currentQuat.w);

        // Calculate quaternion derivative: q_dot = 0.5 * q * omega_quat
        Vector4 quatDot = QuaternionMultiply(currentQuatVec, omega) * 0.5f;

        // Update quaternion
        Vector4 newQuatVec = currentQuatVec + quatDot * dt;

        // Normalize quaternion
        Vector4 normalizedQuat = newQuatVec.normalized;

        orientation = new Quaternion(normalizedQuat.x, normalizedQuat.y, normalizedQuat.z, normalizedQuat.w);

        previousOrientation = orientation;
    }

    Vector4 QuaternionMultiply(Vector4 q1, Vector4 q2)
    {
        // Multiply two quaternions
        float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
        float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
        float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

        return new Vector4(x, y, z, w);
    }

    void PublishIMUData()
    {
        // In a real implementation, this would publish the data to other systems
        // For this example, we'll just log it or send it to a data collector
        IMUDataCollector collector = FindObjectOfType<IMUDataCollector>();
        if (collector != null)
        {
            collector.ReceiveIMUData(linearAcceleration, angularVelocity, magneticField, orientation);
        }
    }

    public Vector3 GetLinearAcceleration()
    {
        return linearAcceleration;
    }

    public Vector3 GetAngularVelocity()
    {
        return angularVelocity;
    }

    public Vector3 GetMagneticField()
    {
        return magneticField;
    }

    public Quaternion GetOrientation()
    {
        return orientation;
    }

    public Vector3 GetEulerAngles()
    {
        return orientation.eulerAngles;
    }

    public void ResetBiases()
    {
        accelerometerBias = Vector3.zero;
        gyroscopeBias = Vector3.zero;
        magnetometerBias = Vector3.zero;
    }

    public void Recalibrate()
    {
        // Recalibrate by recalculating biases
        accelerometerBias = new Vector3(
            UnityEngine.Random.Range(-accelerometerBiasStdDev/10, accelerometerBiasStdDev/10),
            UnityEngine.Random.Range(-accelerometerBiasStdDev/10, accelerometerBiasStdDev/10),
            UnityEngine.Random.Range(-accelerometerBiasStdDev/10, accelerometerBiasStdDev/10)
        );

        gyroscopeBias = new Vector3(
            UnityEngine.Random.Range(-gyroscopeBiasStdDev/10, gyroscopeBiasStdDev/10),
            UnityEngine.Random.Range(-gyroscopeBiasStdDev/10, gyroscopeBiasStdDev/10),
            UnityEngine.Random.Range(-gyroscopeBiasStdDev/10, gyroscopeBiasStdDev/10)
        );

        magnetometerBias = new Vector3(
            UnityEngine.Random.Range(-magnetometerBiasStdDev/10, magnetometerBiasStdDev/10),
            UnityEngine.Random.Range(-magnetometerBiasStdDev/10, magnetometerBiasStdDev/10),
            UnityEngine.Random.Range(-magnetometerBiasStdDev/10, magnetometerBiasStdDev/10)
        );
    }
}
```

### Step 2: Creating an IMU Data Collector

Create a script to collect and process IMU data:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class IMUDataCollector : MonoBehaviour
{
    [Header("Data Collection")]
    [SerializeField] private int maxDataPoints = 1000;
    [SerializeField] private bool collectData = true;
    [SerializeField] private bool visualizeData = true;

    [Header("Integration Parameters")]
    [SerializeField] private bool integrateVelocity = true;
    [SerializeField] private bool integratePosition = true;

    // Data storage
    private Queue<IMUReading> imuDataQueue = new Queue<IMUReading>();
    private Vector3 integratedVelocity = Vector3.zero;
    private Vector3 integratedPosition = Vector3.zero;
    private float lastTime = 0f;

    void Start()
    {
        lastTime = Time.time;
    }

    void Update()
    {
        if (collectData)
        {
            ProcessData();
        }

        if (visualizeData)
        {
            VisualizeData();
        }
    }

    public void ReceiveIMUData(Vector3 linearAcc, Vector3 angularVel, Vector3 magField, Quaternion orientation)
    {
        IMUReading reading = new IMUReading
        {
            timestamp = Time.time,
            linearAcceleration = linearAcc,
            angularVelocity = angularVel,
            magneticField = magField,
            orientation = orientation
        };

        // Add to queue, removing oldest if at max capacity
        if (imuDataQueue.Count >= maxDataPoints)
        {
            imuDataQueue.Dequeue();
        }

        imuDataQueue.Enqueue(reading);
    }

    void ProcessData()
    {
        if (imuDataQueue.Count == 0) return;

        IMUReading latest = imuDataQueue.Peek();

        // Calculate time difference
        float dt = latest.timestamp - lastTime;
        lastTime = latest.timestamp;

        if (integrateVelocity)
        {
            // Integrate acceleration to get velocity
            integratedVelocity += latest.linearAcceleration * dt;
        }

        if (integratePosition)
        {
            // Integrate velocity to get position
            integratedPosition += integratedVelocity * dt;
        }
    }

    void VisualizeData()
    {
        if (imuDataQueue.Count == 0) return;

        IMUReading latest = imuDataQueue.Peek();

        // Visualize acceleration as arrows
        Vector3 worldAcc = transform.TransformDirection(latest.linearAcceleration);
        Debug.DrawRay(transform.position, worldAcc * 0.1f, Color.red);

        // Visualize angular velocity as rotation indicator
        Vector3 worldAngVel = transform.TransformDirection(latest.angularVelocity);
        Debug.DrawRay(transform.position, worldAngVel * 0.05f, Color.green);

        // Log important values
        if (Time.frameCount % 60 == 0) // Log every 60 frames
        {
            Debug.Log($"IMU Data - Acc: {latest.linearAcceleration}, Vel: {latest.angularVelocity}, Vel integrated: {integratedVelocity}, Pos integrated: {integratedPosition}");
        }
    }

    public List<IMUReading> GetRecentData(int count)
    {
        List<IMUReading> data = new List<IMUReading>();
        Queue<IMUReading> tempQueue = new Queue<IMUReading>(imuDataQueue);

        int itemsToTake = Mathf.Min(count, tempQueue.Count);
        for (int i = 0; i < itemsToTake; i++)
        {
            data.Add(tempQueue.Dequeue());
        }

        return data;
    }

    public Vector3 GetIntegratedVelocity()
    {
        return integratedVelocity;
    }

    public Vector3 GetIntegratedPosition()
    {
        return integratedPosition;
    }

    public IMUReading GetLatestReading()
    {
        if (imuDataQueue.Count == 0) return new IMUReading();
        return imuDataQueue.Peek();
    }

    public void ClearData()
    {
        imuDataQueue.Clear();
        integratedVelocity = Vector3.zero;
        integratedPosition = Vector3.zero;
    }
}

[System.Serializable]
public struct IMUReading
{
    public float timestamp;
    public Vector3 linearAcceleration;
    public Vector3 angularVelocity;
    public Vector3 magneticField;
    public Quaternion orientation;
}
```

## Processing IMU Data

### Step 1: Creating an IMU Data Processing Pipeline

Create a script to process IMU data for robotics applications:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class IMUProcessor : MonoBehaviour
{
    [Header("Processing Configuration")]
    [SerializeField] private UnityIMU imuSource;
    [SerializeField] private float accelerometerCutoff = 0.1f;
    [SerializeField] private float gyroscopeCutoff = 0.1f;
    [SerializeField] private bool useComplementaryFilter = true;
    [SerializeField] private float complementaryFilterAlpha = 0.98f;

    [Header("State Estimation")]
    [SerializeField] private bool estimatePose = true;
    [SerializeField] private bool estimateVelocity = true;
    [SerializeField] private bool detectMotion = true;

    // Internal state
    private Vector3 filteredAcceleration;
    private Vector3 filteredAngularVelocity;
    private Vector3 estimatedVelocity;
    private Vector3 estimatedPosition;
    private Quaternion estimatedOrientation;
    private bool isMoving = false;
    private float motionThreshold = 0.1f;

    // Previous values for integration
    private float previousTime = 0f;
    private Vector3 previousVelocity = Vector3.zero;
    private Vector3 previousPosition = Vector3.zero;
    private Quaternion previousOrientation = Quaternion.identity;

    void Start()
    {
        InitializeProcessor();
    }

    void InitializeProcessor()
    {
        filteredAcceleration = Vector3.zero;
        filteredAngularVelocity = Vector3.zero;
        estimatedVelocity = Vector3.zero;
        estimatedPosition = Vector3.zero;
        estimatedOrientation = Quaternion.identity;
        previousTime = Time.time;
    }

    void Update()
    {
        if (imuSource != null)
        {
            ProcessIMUData();
        }
    }

    void ProcessIMUData()
    {
        Vector3 rawAcceleration = imuSource.GetLinearAcceleration();
        Vector3 rawAngularVelocity = imuSource.GetAngularVelocity();
        Quaternion rawOrientation = imuSource.GetOrientation();

        float currentTime = Time.time;
        float deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        // Apply filtering to raw data
        ApplyFilters(rawAcceleration, rawAngularVelocity);

        // Estimate state if enabled
        if (estimatePose)
        {
            EstimatePose(rawOrientation, deltaTime);
        }

        if (estimateVelocity)
        {
            EstimateVelocity(deltaTime);
        }

        if (detectMotion)
        {
            DetectMotion();
        }

        // Store current values for next iteration
        previousVelocity = estimatedVelocity;
        previousPosition = estimatedPosition;
        previousOrientation = estimatedOrientation;
    }

    void ApplyFilters(Vector3 acceleration, Vector3 angularVelocity)
    {
        // Simple low-pass filtering
        if (useComplementaryFilter)
        {
            // Complementary filter combines accelerometer and gyroscope data
            // This is a simplified approach - in practice, you'd use more sophisticated filtering
            filteredAcceleration = Vector3.Lerp(filteredAcceleration, acceleration, 0.1f);
            filteredAngularVelocity = Vector3.Lerp(filteredAngularVelocity, angularVelocity, 0.1f);
        }
        else
        {
            // Simple averaging filter
            filteredAcceleration = Vector3.Lerp(filteredAcceleration, acceleration, 0.2f);
            filteredAngularVelocity = Vector3.Lerp(filteredAngularVelocity, angularVelocity, 0.2f);
        }
    }

    void EstimatePose(Quaternion rawOrientation, float deltaTime)
    {
        if (deltaTime > 0)
        {
            // Integrate angular velocity to update orientation
            Vector3 angularVelocityRad = filteredAngularVelocity * Mathf.Deg2Rad;
            float angle = angularVelocityRad.magnitude * deltaTime;

            if (angle > 0.001f) // Avoid division by zero
            {
                Vector3 axis = angularVelocityRad.normalized;
                Quaternion rotationIncrement = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
                estimatedOrientation = rotationIncrement * previousOrientation;
            }
            else
            {
                estimatedOrientation = previousOrientation;
            }
        }
        else
        {
            estimatedOrientation = rawOrientation; // Use raw orientation if no time has passed
        }
    }

    void EstimateVelocity(float deltaTime)
    {
        if (deltaTime > 0)
        {
            // Integrate acceleration to get velocity
            // Note: This is a simplified approach and would accumulate error over time
            estimatedVelocity += filteredAcceleration * deltaTime;

            // Apply motion threshold to reduce drift
            if (estimatedVelocity.magnitude < motionThreshold)
            {
                estimatedVelocity = Vector3.zero;
            }
        }
    }

    void EstimatePosition(float deltaTime)
    {
        if (deltaTime > 0)
        {
            // Integrate velocity to get position
            estimatedPosition += estimatedVelocity * deltaTime;
        }
    }

    void DetectMotion()
    {
        // Simple motion detection based on acceleration magnitude
        float accelerationMagnitude = filteredAcceleration.magnitude;
        isMoving = accelerationMagnitude > motionThreshold;
    }

    public Vector3 GetFilteredAcceleration()
    {
        return filteredAcceleration;
    }

    public Vector3 GetFilteredAngularVelocity()
    {
        return filteredAngularVelocity;
    }

    public Vector3 GetEstimatedVelocity()
    {
        return estimatedVelocity;
    }

    public Vector3 GetEstimatedPosition()
    {
        return estimatedPosition;
    }

    public Quaternion GetEstimatedOrientation()
    {
        return estimatedOrientation;
    }

    public bool IsMoving()
    {
        return isMoving;
    }

    public float GetAccelerationMagnitude()
    {
        return filteredAcceleration.magnitude;
    }

    public float GetAngularVelocityMagnitude()
    {
        return filteredAngularVelocity.magnitude;
    }

    public void ResetEstimation()
    {
        estimatedVelocity = Vector3.zero;
        estimatedPosition = Vector3.zero;
        estimatedOrientation = Quaternion.identity;
        previousVelocity = Vector3.zero;
        previousPosition = Vector3.zero;
        previousOrientation = Quaternion.identity;
    }

    // Visualization for debugging
    void OnDrawGizmos()
    {
        if (estimatePose)
        {
            // Draw estimated orientation
            Vector3 position = transform.position;
            Vector3 forward = estimatedOrientation * Vector3.forward;
            Vector3 up = estimatedOrientation * Vector3.up;
            Vector3 right = estimatedOrientation * Vector3.right;

            Gizmos.color = Color.blue;
            Gizmos.DrawRay(position, forward * 0.5f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(position, up * 0.5f);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(position, right * 0.5f);
        }
    }
}
```

## Validation and Testing

### Step 1: IMU Data Validation

Create a validation script to ensure your IMU simulation is accurate:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class IMUValidator : MonoBehaviour
{
    [Header("Validation Configuration")]
    [SerializeField] private UnityIMU imuSource;
    [SerializeField] private Transform referenceObject;
    [SerializeField] private float accelerationTolerance = 0.1f;
    [SerializeField] private float angularVelocityTolerance = 0.05f;
    [SerializeField] private float orientationTolerance = 5f; // degrees
    [SerializeField] private float accuracyThreshold = 0.95f;

    [Header("Validation Results")]
    [SerializeField] private float currentAccuracy = 0f;
    [SerializeField] private int totalValidations = 0;
    [SerializeField] private int successfulValidations = 0;
    [SerializeField] private float averageError = 0f;

    private List<float> validationErrors = new List<float>();
    private float nextValidation = 0f;
    private float validationInterval = 1f; // Validate every second

    void Update()
    {
        if (Time.time >= nextValidation)
        {
            PerformValidation();
            nextValidation = Time.time + validationInterval;
        }
    }

    void PerformValidation()
    {
        if (imuSource == null || referenceObject == null) return;

        // Get IMU data
        Vector3 simulatedAcceleration = imuSource.GetLinearAcceleration();
        Vector3 simulatedAngularVelocity = imuSource.GetAngularVelocity();
        Quaternion simulatedOrientation = imuSource.GetOrientation();

        // Calculate expected values based on reference object
        Vector3 expectedAcceleration = CalculateExpectedAcceleration();
        Vector3 expectedAngularVelocity = CalculateExpectedAngularVelocity();
        Quaternion expectedOrientation = CalculateExpectedOrientation();

        // Compare simulated vs expected values
        float accelerationError = Vector3.Distance(simulatedAcceleration, expectedAcceleration);
        float angularVelocityError = Vector3.Distance(simulatedAngularVelocity, expectedAngularVelocity);
        float orientationError = Quaternion.Angle(simulatedOrientation, expectedOrientation);

        // Calculate total error (weighted combination of all errors)
        float totalError = (accelerationError / accelerationTolerance) * 0.4f +
                          (angularVelocityError / angularVelocityTolerance) * 0.4f +
                          (orientationError / orientationTolerance) * 0.2f;

        // Store validation result
        validationErrors.Add(totalError);
        totalValidations++;

        if (totalError <= 1.0f) // Within tolerance (normalized to 1.0)
        {
            successfulValidations++;
        }

        // Calculate current accuracy
        currentAccuracy = (float)successfulValidations / totalValidations;

        // Calculate average error
        averageError = validationErrors.Count > 0 ? validationErrors.Average() : 0f;

        // Log validation results
        if (currentAccuracy < accuracyThreshold)
        {
            Debug.LogWarning($"IMU accuracy below threshold: {currentAccuracy:P2} (threshold: {accuracyThreshold:P2}), avg error: {averageError:F3}");
        }
        else
        {
            Debug.Log($"IMU accuracy: {currentAccuracy:P2}, avg error: {averageError:F3}");
        }

        // Keep only recent validation results to prevent memory issues
        if (validationErrors.Count > 100)
        {
            validationErrors.RemoveRange(0, 50);
        }
    }

    Vector3 CalculateExpectedAcceleration()
    {
        // Calculate expected acceleration based on reference object's motion
        Rigidbody rb = referenceObject.GetComponent<Rigidbody>();
        if (rb != null)
        {
            // Use the rigidbody's acceleration
            // Note: Unity doesn't directly expose acceleration, so we estimate it
            return (rb.velocity - rb.velocity) / Time.deltaTime; // This would be from previous frame
        }

        // Fallback: calculate from position changes
        return Vector3.zero; // Simplified - in practice, you'd track position over time
    }

    Vector3 CalculateExpectedAngularVelocity()
    {
        // Calculate expected angular velocity based on reference object's rotation
        Rigidbody rb = referenceObject.GetComponent<Rigidbody>();
        if (rb != null)
        {
            return rb.angularVelocity;
        }

        // Fallback: calculate from rotation changes
        return Vector3.zero; // Simplified
    }

    Quaternion CalculateExpectedOrientation()
    {
        // Expected orientation is the same as the reference object's orientation
        return referenceObject.rotation;
    }

    public float GetAccuracy()
    {
        return currentAccuracy;
    }

    public float GetAverageError()
    {
        return averageError;
    }

    public bool IsAccuracyValid()
    {
        return currentAccuracy >= accuracyThreshold;
    }

    public void ResetValidation()
    {
        totalValidations = 0;
        successfulValidations = 0;
        validationErrors.Clear();
        currentAccuracy = 0f;
        averageError = 0f;
    }
}
```

## Exercise: IMU Implementation

1. Implement an IMU sensor on your humanoid robot model in both Gazebo and Unity
2. Configure the IMU with realistic parameters (ranges, update rates, noise characteristics)
3. Add noise models and bias parameters to make the simulation more realistic
4. Create an IMU data processing pipeline with filtering and state estimation
5. Implement sensor fusion techniques to combine IMU data with other sensors
6. Validate your IMU simulation against known motion patterns
7. Document the accuracy of your simulation and identify areas for improvement

## Troubleshooting Common Issues

### Issue: IMU data is too noisy
**Solution**: Implement proper filtering (complementary filter, Kalman filter) and adjust noise parameters.

### Issue: Orientation drift over time
**Solution**: Implement proper bias estimation and correction, or combine with other sensors for absolute orientation reference.

### Issue: Integration errors accumulating
**Solution**: Implement proper drift correction algorithms and use more sophisticated integration techniques.

## Next Steps

After implementing IMU simulation, proceed to [Sensor Integration Guide](./sensor-integration.md) to learn how to combine multiple sensors in your digital twin environment.