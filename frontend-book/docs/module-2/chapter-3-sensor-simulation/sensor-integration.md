# Sensor Integration Guide

## Overview

This guide covers the integration of multiple sensors in digital twin environments, specifically how to combine LiDAR, depth camera, and IMU sensors to create a comprehensive perception system. You'll learn how to synchronize, calibrate, and fuse data from multiple sensors for enhanced robotics applications.

## Prerequisites

- Understanding of individual sensor simulation (LiDAR, depth camera, IMU)
- Basic knowledge of Gazebo and Unity
- Completion of previous sensor simulation tutorials
- Understanding of coordinate systems and transformations

## Multi-Sensor Architecture

### Sensor Data Flow

In a multi-sensor system, data flows from individual sensors through various processing stages:

```
Sensor 1 → Data Acquisition → Preprocessing → Synchronization → Fusion → Post-processing → Application
Sensor 2 → Data Acquisition → Preprocessing → Synchronization → Fusion → Post-processing → Application
Sensor 3 → Data Acquisition → Preprocessing → Synchronization → Fusion → Post-processing → Application
```

### Key Integration Challenges

1. **Temporal Synchronization**: Ensuring all sensors capture data simultaneously
2. **Spatial Calibration**: Aligning sensor coordinate systems
3. **Data Fusion**: Combining data from multiple sources effectively
4. **Resource Management**: Optimizing computational resources for multiple sensors

## Coordinate System Alignment

### Understanding Sensor Frames

Each sensor operates in its own coordinate frame. For proper integration, all sensor data must be transformed to a common reference frame.

**Common Robot Frames:**
- **Base Frame**: Robot's main reference frame (usually at the center of mass)
- **Sensor Frames**: Individual frames for each sensor
- **World Frame**: Global reference frame for the environment

### Transform Configuration in URDF

Configure transforms between sensors in your URDF model:

```xml
<!-- Robot base link -->
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
  </visual>
</link>

<!-- IMU sensor -->
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
  </visual>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
</joint>

<!-- LiDAR sensor -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.1 0.0 0.3" rpy="0 0 0"/>
</joint>

<!-- Depth camera -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.03 0.02"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.15 0.0 0.2" rpy="0 0 0"/>
</joint>
```

### Transform Configuration in Unity

In Unity, use GameObject hierarchy to represent transforms:

```csharp
using UnityEngine;

public class SensorTransformManager : MonoBehaviour
{
    [Header("Sensor Mounting Points")]
    [SerializeField] private Transform imuMountPoint;
    [SerializeField] private Transform lidarMountPoint;
    [SerializeField] private Transform cameraMountPoint;

    [Header("Calibration Offsets")]
    [SerializeField] private Vector3 imuOffset = Vector3.zero;
    [SerializeField] private Vector3 lidarOffset = Vector3.zero;
    [SerializeField] private Vector3 cameraOffset = Vector3.zero;

    [SerializeField] private Vector3 imuRotation = Vector3.zero;
    [SerializeField] private Vector3 lidarRotation = Vector3.zero;
    [SerializeField] private Vector3 cameraRotation = Vector3.zero;

    private void Start()
    {
        SetupSensorTransforms();
    }

    private void SetupSensorTransforms()
    {
        // Apply calibration offsets to each sensor
        if (imuMountPoint != null)
        {
            imuMountPoint.localPosition = imuOffset;
            imuMountPoint.localEulerAngles = imuRotation;
        }

        if (lidarMountPoint != null)
        {
            lidarMountPoint.localPosition = lidarOffset;
            lidarMountPoint.localEulerAngles = lidarRotation;
        }

        if (cameraMountPoint != null)
        {
            cameraMountPoint.localPosition = cameraOffset;
            cameraMountPoint.localEulerAngles = cameraRotation;
        }
    }

    public Vector3 GetSensorPosition(string sensorName)
    {
        switch (sensorName.ToLower())
        {
            case "imu":
                return imuMountPoint != null ? imuMountPoint.position : Vector3.zero;
            case "lidar":
                return lidarMountPoint != null ? lidarMountPoint.position : Vector3.zero;
            case "camera":
                return cameraMountPoint != null ? cameraMountPoint.position : Vector3.zero;
            default:
                return Vector3.zero;
        }
    }

    public Quaternion GetSensorRotation(string sensorName)
    {
        switch (sensorName.ToLower())
        {
            case "imu":
                return imuMountPoint != null ? imuMountPoint.rotation : Quaternion.identity;
            case "lidar":
                return lidarMountPoint != null ? lidarMountPoint.rotation : Quaternion.identity;
            case "camera":
                return cameraMountPoint != null ? cameraMountPoint.rotation : Quaternion.identity;
            default:
                return Quaternion.identity;
        }
    }
}
```

## Temporal Synchronization

### Understanding Timestamps

Each sensor produces data with a timestamp. For proper fusion, data from all sensors must be synchronized to a common time reference.

### ROS 2 Time Synchronization

In ROS 2, use message filters for temporal synchronization:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import numpy as np

class MultiSensorSynchronizer(Node):
    def __init__(self):
        super().__init__('multi_sensor_synchronizer')

        # Create subscribers for each sensor
        lidar_sub = Subscriber(self, PointCloud2, '/lidar/points')
        camera_sub = Subscriber(self, Image, '/camera/rgb/image_raw')
        imu_sub = Subscriber(self, Imu, '/imu/data')

        # Create approximate time synchronizer
        self.ts = ApproximateTimeSynchronizer(
            [lidar_sub, camera_sub, imu_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.ts.registerCallback(self.sync_callback)

        self.bridge = CvBridge()
        self.get_logger().info('Multi-sensor synchronizer initialized')

    def sync_callback(self, lidar_msg, camera_msg, imu_msg):
        # All messages are now synchronized in time
        self.get_logger().info(f'Synchronized at time: {lidar_msg.header.stamp}')

        # Process synchronized data
        self.process_lidar_data(lidar_msg)
        self.process_camera_data(camera_msg)
        self.process_imu_data(imu_msg)

        # Perform fusion with synchronized data
        self.perform_sensor_fusion(lidar_msg, camera_msg, imu_msg)

    def process_lidar_data(self, lidar_msg):
        # Extract point cloud data
        points = list(point_cloud2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True))
        self.get_logger().info(f'Processed {len(points)} lidar points')

    def process_camera_data(self, camera_msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        self.get_logger().info(f'Processed camera image: {cv_image.shape}')

    def process_imu_data(self, imu_msg):
        # Extract IMU data
        linear_acc = imu_msg.linear_acceleration
        angular_vel = imu_msg.angular_velocity
        orientation = imu_msg.orientation
        self.get_logger().info(f'IMU - Acc: ({linear_acc.x:.3f}, {linear_acc.y:.3f}, {linear_acc.z:.3f})')

    def perform_sensor_fusion(self, lidar_msg, camera_msg, imu_msg):
        # Example fusion: use IMU to improve LiDAR pose estimation
        # This is a simplified example - real fusion would be more complex

        # Get IMU orientation for pose correction
        imu_orientation = [imu_msg.orientation.x, imu_msg.orientation.y,
                          imu_msg.orientation.z, imu_msg.orientation.w]

        # Apply IMU data to improve LiDAR point cloud alignment
        # This would involve transforming points based on IMU orientation
        self.get_logger().info('Sensor fusion performed')

def main(args=None):
    rclpy.init(args=args)
    synchronizer = MultiSensorSynchronizer()

    try:
        rclpy.spin(synchronizer)
    except KeyboardInterrupt:
        pass
    finally:
        synchronizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Unity Temporal Synchronization

In Unity, implement a time synchronization system:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class UnitySensorSynchronizer : MonoBehaviour
{
    [Header("Synchronization Settings")]
    [SerializeField] private float syncTolerance = 0.05f; // 50ms tolerance
    [SerializeField] private int maxQueueSize = 100;

    [Header("Sensor Sources")]
    [SerializeField] private UnityLiDAR lidarSource;
    [SerializeField] private UnityDepthCamera cameraSource;
    [SerializeField] private UnityIMU imuSource;

    private Queue<SensorData> lidarQueue = new Queue<SensorData>();
    private Queue<SensorData> cameraQueue = new Queue<SensorData>();
    private Queue<SensorData> imuQueue = new Queue<SensorData>();

    private List<SynchronizedData> synchronizedData = new List<SynchronizedData>();

    void Update()
    {
        // Add current data to queues
        AddToQueue(lidarQueue, GetLiDARData(), Time.time);
        AddToQueue(cameraQueue, GetCameraData(), Time.time);
        AddToQueue(imuQueue, GetIMUData(), Time.time);

        // Attempt to synchronize data
        SynchronizeData();
    }

    void AddToQueue(Queue<SensorData> queue, object data, float timestamp)
    {
        SensorData sensorData = new SensorData
        {
            data = data,
            timestamp = timestamp
        };

        queue.Enqueue(sensorData);

        // Limit queue size
        if (queue.Count > maxQueueSize)
        {
            queue.Dequeue();
        }
    }

    void SynchronizeData()
    {
        // Find synchronized data triplets
        while (lidarQueue.Count > 0 && cameraQueue.Count > 0 && imuQueue.Count > 0)
        {
            SensorData lidarData = lidarQueue.Peek();
            SensorData cameraData = cameraQueue.Peek();
            SensorData imuData = imuQueue.Peek();

            // Check if timestamps are within tolerance
            float timeDiff1 = Mathf.Abs(lidarData.timestamp - cameraData.timestamp);
            float timeDiff2 = Mathf.Abs(lidarData.timestamp - imuData.timestamp);
            float timeDiff3 = Mathf.Abs(cameraData.timestamp - imuData.timestamp);

            if (timeDiff1 <= syncTolerance &&
                timeDiff2 <= syncTolerance &&
                timeDiff3 <= syncTolerance)
            {
                // All three sensors are synchronized
                SynchronizedData syncData = new SynchronizedData
                {
                    lidarData = lidarQueue.Dequeue().data,
                    cameraData = cameraQueue.Dequeue().data,
                    imuData = imuQueue.Dequeue().data,
                    timestamp = (lidarData.timestamp + cameraData.timestamp + imuData.timestamp) / 3f
                };

                synchronizedData.Add(syncData);
                ProcessSynchronizedData(syncData);

                // Limit synchronized data storage
                if (synchronizedData.Count > maxQueueSize)
                {
                    synchronizedData.RemoveAt(0);
                }
            }
            else
            {
                // Find the earliest timestamp and remove that data
                float earliest = Mathf.Min(lidarData.timestamp, cameraData.timestamp, imuData.timestamp);

                if (Mathf.Abs(lidarData.timestamp - earliest) < 0.001f)
                    lidarQueue.Dequeue();
                else if (Mathf.Abs(cameraData.timestamp - earliest) < 0.001f)
                    cameraQueue.Dequeue();
                else
                    imuQueue.Dequeue();
            }
        }
    }

    object GetLiDARData()
    {
        if (lidarSource != null)
        {
            return lidarSource.GetPointCloud();
        }
        return null;
    }

    object GetCameraData()
    {
        if (cameraSource != null)
        {
            return cameraSource.GetDepthData();
        }
        return null;
    }

    object GetIMUData()
    {
        if (imuSource != null)
        {
            return new IMUData
            {
                linearAcceleration = imuSource.GetLinearAcceleration(),
                angularVelocity = imuSource.GetAngularVelocity(),
                orientation = imuSource.GetOrientation()
            };
        }
        return null;
    }

    void ProcessSynchronizedData(SynchronizedData syncData)
    {
        // Perform sensor fusion with synchronized data
        Debug.Log($"Synchronized data at {syncData.timestamp:F3}s");

        // Example: Use IMU data to improve LiDAR point cloud alignment
        if (syncData.lidarData is List<Vector3> lidarPoints &&
            syncData.imuData is IMUData imuData)
        {
            // Apply IMU-based correction to LiDAR points
            ApplyIMUMotionCorrection(lidarPoints, imuData);
        }
    }

    void ApplyIMUMotionCorrection(List<Vector3> points, IMUData imuData)
    {
        // Simple example: adjust points based on IMU acceleration
        // In practice, this would involve more complex motion compensation
        for (int i = 0; i < points.Count; i++)
        {
            // Apply small correction based on IMU data
            Vector3 correction = imuData.linearAcceleration * 0.001f; // Scale factor
            points[i] += correction;
        }
    }
}

[System.Serializable]
public struct SensorData
{
    public object data;
    public float timestamp;
}

[System.Serializable]
public struct SynchronizedData
{
    public object lidarData;
    public object cameraData;
    public object imuData;
    public float timestamp;
}

[System.Serializable]
public struct IMUData
{
    public Vector3 linearAcceleration;
    public Vector3 angularVelocity;
    public Quaternion orientation;
}
```

## Data Fusion Techniques

### Sensor Fusion Overview

Data fusion combines data from multiple sensors to produce more accurate, complete, and reliable information than any individual sensor could provide.

### Kalman Filter Implementation

Implement a simple Kalman filter for sensor fusion:

```csharp
using UnityEngine;

public class KalmanFilter
{
    private Matrix4x4 state;        // State vector [position, velocity]
    private Matrix4x4 covariance;   // Error covariance matrix
    private Matrix4x4 processNoise; // Process noise covariance
    private Matrix4x4 measurementNoise; // Measurement noise covariance
    private Matrix4x4 measurementMatrix; // Measurement matrix
    private Matrix4x4 identity;     // Identity matrix

    public KalmanFilter()
    {
        // Initialize with default values
        state = Matrix4x4.zero;
        covariance = Matrix4x4.identity;
        processNoise = Matrix4x4.identity * 0.1f;
        measurementNoise = Matrix4x4.identity * 0.1f;
        measurementMatrix = Matrix4x4.identity;
        identity = Matrix4x4.identity;
    }

    public void Predict(float deltaTime)
    {
        // State transition matrix (simple constant velocity model)
        Matrix4x4 F = Matrix4x4.identity;
        F[0, 3] = deltaTime; // Position changes with velocity

        // Predict state
        state = F * state;

        // Predict covariance
        Matrix4x4 Ft = Transpose(F);
        covariance = F * covariance * Ft + processNoise;
    }

    public void Update(Vector3 measurement)
    {
        // Calculate Kalman gain
        Matrix4x4 S = measurementMatrix * covariance * Transpose(measurementMatrix) + measurementNoise;
        Matrix4x4 K = covariance * Transpose(measurementMatrix) * Inverse(S);

        // Calculate innovation
        Vector4 innovation = new Vector4(measurement.x, measurement.y, measurement.z, 0) -
                           (Vector4)(measurementMatrix * state);

        // Update state
        state = state + K * innovation;

        // Update covariance
        Matrix4x4 I_KH = identity - K * measurementMatrix;
        covariance = I_KH * covariance;
    }

    Matrix4x4 Transpose(Matrix4x4 m)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = m[j, i];
            }
        }
        return result;
    }

    Matrix4x4 Inverse(Matrix4x4 m)
    {
        // For a simple implementation, we'll return the original matrix
        // In practice, you'd implement a proper matrix inverse function
        return m; // This is a placeholder
    }
}
```

### Multi-Sensor Fusion in Unity

Create a comprehensive fusion system:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class MultiSensorFusion : MonoBehaviour
{
    [Header("Sensor Sources")]
    [SerializeField] private UnityLiDAR lidarSource;
    [SerializeField] private UnityDepthCamera cameraSource;
    [SerializeField] private UnityIMU imuSource;
    [SerializeField] private UnitySensorSynchronizer synchronizer;

    [Header("Fusion Parameters")]
    [SerializeField] private float lidarWeight = 0.4f;
    [SerializeField] private float cameraWeight = 0.3f;
    [SerializeField] private float imuWeight = 0.3f;
    [SerializeField] private bool useKalmanFilter = true;

    private KalmanFilter kalmanFilter;
    private List<Vector3> fusedPointCloud = new List<Vector3>();
    private Vector3 fusedPosition = Vector3.zero;
    private Quaternion fusedOrientation = Quaternion.identity;

    void Start()
    {
        if (useKalmanFilter)
        {
            kalmanFilter = new KalmanFilter();
        }
    }

    void Update()
    {
        PerformSensorFusion();
    }

    void PerformSensorFusion()
    {
        // Get data from all sensors
        List<Vector3> lidarData = GetLiDARData();
        float[,] cameraData = GetCameraData();
        IMUData imuData = GetIMUData();

        if (lidarData != null && cameraData != null)
        {
            // Fuse sensor data
            fusedPointCloud = FusePointCloudData(lidarData, cameraData);
            fusedPosition = FusePositionData(imuData.linearAcceleration, lidarData);
            fusedOrientation = FuseOrientationData(imuData.orientation, cameraData);
        }
    }

    List<Vector3> GetLiDARData()
    {
        if (lidarSource != null)
        {
            return lidarSource.GetPointCloud();
        }
        return new List<Vector3>();
    }

    float[,] GetCameraData()
    {
        if (cameraSource != null)
        {
            return cameraSource.GetDepthData();
        }
        return null;
    }

    IMUData GetIMUData()
    {
        if (imuSource != null)
        {
            return new IMUData
            {
                linearAcceleration = imuSource.GetLinearAcceleration(),
                angularVelocity = imuSource.GetAngularVelocity(),
                orientation = imuSource.GetOrientation()
            };
        }
        return new IMUData();
    }

    List<Vector3> FusePointCloudData(List<Vector3> lidarPoints, float[,] cameraDepth)
    {
        // Combine LiDAR and camera depth data
        List<Vector3> fusedPoints = new List<Vector3>();

        // This is a simplified approach - real fusion would be more complex
        foreach (Vector3 lidarPoint in lidarPoints)
        {
            // Add LiDAR points with confidence based on fusion weights
            if (Random.value < lidarWeight)
            {
                fusedPoints.Add(lidarPoint);
            }
        }

        // Add points from camera depth data if needed
        if (cameraDepth != null)
        {
            int width = cameraDepth.GetLength(0);
            int height = cameraDepth.GetLength(1);

            // Sample points from depth image
            for (int y = 0; y < height; y += 10) // Subsample for performance
            {
                for (int x = 0; x < width; x += 10)
                {
                    float depth = cameraDepth[x, y];
                    if (depth > 0 && Random.value < cameraWeight)
                    {
                        Vector3 cameraPoint = Camera.main.ViewportToWorldPoint(
                            new Vector3((float)x / width, (float)y / height, depth)
                        );
                        fusedPoints.Add(cameraPoint);
                    }
                }
            }
        }

        return fusedPoints;
    }

    Vector3 FusePositionData(Vector3 imuAcceleration, List<Vector3> lidarPoints)
    {
        // Simple weighted fusion of position data
        Vector3 imuContribution = imuAcceleration * imuWeight * Time.deltaTime;

        // Calculate position from LiDAR data (simplified)
        Vector3 lidarContribution = Vector3.zero;
        if (lidarPoints.Count > 0)
        {
            foreach (Vector3 point in lidarPoints)
            {
                lidarContribution += point;
            }
            lidarContribution /= lidarPoints.Count;
            lidarContribution *= lidarWeight;
        }

        return imuContribution + lidarContribution;
    }

    Quaternion FuseOrientationData(Quaternion imuOrientation, float[,] cameraDepth)
    {
        // For now, use IMU orientation as primary source
        // In practice, you'd combine with visual odometry from camera
        return imuOrientation;
    }

    public List<Vector3> GetFusedPointCloud()
    {
        return new List<Vector3>(fusedPointCloud);
    }

    public Vector3 GetFusedPosition()
    {
        return fusedPosition;
    }

    public Quaternion GetFusedOrientation()
    {
        return fusedOrientation;
    }

    // Visualization for debugging
    void OnDrawGizmos()
    {
        if (fusedPointCloud.Count > 0)
        {
            Gizmos.color = Color.magenta;
            foreach (Vector3 point in fusedPointCloud)
            {
                Gizmos.DrawSphere(point, 0.02f);
            }
        }

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(fusedPosition, Vector3.one * 0.1f);
    }
}
```

## Calibration Procedures

### Sensor Calibration Overview

Proper calibration is essential for accurate sensor fusion. Calibration involves determining the geometric relationship between sensors and correcting for sensor-specific errors.

### LiDAR-Camera Calibration

Calibrate the relationship between LiDAR and camera:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class LiDARCameraCalibrator : MonoBehaviour
{
    [Header("Calibration Configuration")]
    [SerializeField] private UnityLiDAR lidarSource;
    [SerializeField] private UnityDepthCamera cameraSource;
    [SerializeField] private GameObject calibrationTarget; // Checkerboard or known object

    [Header("Calibration Results")]
    [SerializeField] private Matrix4x4 lidarToCameraTransform = Matrix4x4.identity;
    [SerializeField] private float calibrationError = 0f;

    private List<Vector3> lidarPoints = new List<Vector3>();
    private float[,] cameraDepth = null;
    private bool isCalibrating = false;

    public void StartCalibration()
    {
        isCalibrating = true;
        lidarPoints.Clear();

        // Move to multiple positions to collect calibration data
        StartCoroutine(CalibrationProcess());
    }

    System.Collections.IEnumerator CalibrationProcess()
    {
        for (int i = 0; i < 10; i++) // Collect data from 10 different positions
        {
            // Move calibration target to new position
            MoveCalibrationTarget(i);

            // Wait for sensors to settle
            yield return new WaitForSeconds(1.0f);

            // Collect synchronized data
            CollectCalibrationData();

            yield return null;
        }

        // Perform calibration calculation
        PerformCalibration();
        isCalibrating = false;
    }

    void MoveCalibrationTarget(int step)
    {
        // Move calibration target to different positions for better calibration
        if (calibrationTarget != null)
        {
            float angle = (step / 10.0f) * 360f;
            float radius = 1.0f;
            float x = Mathf.Cos(angle * Mathf.Deg2Rad) * radius;
            float z = Mathf.Sin(angle * Mathf.Deg2Rad) * radius;

            calibrationTarget.transform.position = new Vector3(x, 0.5f, z);
        }
    }

    void CollectCalibrationData()
    {
        if (lidarSource != null)
        {
            lidarPoints.AddRange(lidarSource.GetPointCloud());
        }

        if (cameraSource != null)
        {
            cameraDepth = cameraSource.GetDepthData();
        }
    }

    void PerformCalibration()
    {
        // Simplified calibration - in practice, this would use more sophisticated algorithms
        // like Zhang's method for camera calibration or iterative closest point (ICP) for LiDAR-camera

        // Find corresponding points between LiDAR and camera
        List<Vector3> correspondingLiDAR = new List<Vector3>();
        List<Vector3> correspondingCamera = new List<Vector3>();

        // This is a simplified approach - real calibration would match features
        for (int i = 0; i < Mathf.Min(lidarPoints.Count, 100); i++)
        {
            // Find corresponding point in camera data (simplified)
            Vector3 cameraPoint = ProjectLiDARToCamera(lidarPoints[i]);
            if (cameraPoint != Vector3.zero)
            {
                correspondingLiDAR.Add(lidarPoints[i]);
                correspondingCamera.Add(cameraPoint);
            }
        }

        // Calculate transformation matrix
        if (correspondingLiDAR.Count > 3)
        {
            lidarToCameraTransform = CalculateTransform(correspondingLiDAR, correspondingCamera);
            calibrationError = CalculateCalibrationError(correspondingLiDAR, correspondingCamera);
        }
    }

    Vector3 ProjectLiDARToCamera(Vector3 lidarPoint)
    {
        // Project LiDAR point to camera coordinate system
        Vector3 cameraPoint = lidarToCameraTransform.MultiplyPoint3x4(lidarPoint);
        return cameraPoint;
    }

    Matrix4x4 CalculateTransform(List<Vector3> pointsA, List<Vector3> pointsB)
    {
        // Simplified transform calculation - in practice, use SVD or other methods
        if (pointsA.Count != pointsB.Count || pointsA.Count < 3)
            return Matrix4x4.identity;

        // Calculate centroids
        Vector3 centroidA = Vector3.zero;
        Vector3 centroidB = Vector3.zero;

        foreach (Vector3 p in pointsA) centroidA += p;
        foreach (Vector3 p in pointsB) centroidB += p;

        centroidA /= pointsA.Count;
        centroidB /= pointsB.Count;

        // Calculate covariance matrix
        Matrix4x4 H = Matrix4x4.zero;
        for (int i = 0; i < pointsA.Count; i++)
        {
            Vector3 aCentered = pointsA[i] - centroidA;
            Vector3 bCentered = pointsB[i] - centroidB;

            // Add outer product to H
            H[0, 0] += aCentered.x * bCentered.x;
            H[0, 1] += aCentered.x * bCentered.y;
            H[0, 2] += aCentered.x * bCentered.z;
            H[1, 0] += aCentered.y * bCentered.x;
            H[1, 1] += aCentered.y * bCentered.y;
            H[1, 2] += aCentered.y * bCentered.z;
            H[2, 0] += aCentered.z * bCentered.x;
            H[2, 1] += aCentered.z * bCentered.y;
            H[2, 2] += aCentered.z * bCentered.z;
        }

        // For simplicity, return identity matrix
        // In practice, perform SVD decomposition of H
        return Matrix4x4.identity;
    }

    float CalculateCalibrationError(List<Vector3> pointsA, List<Vector3> pointsB)
    {
        float totalError = 0f;
        for (int i = 0; i < pointsA.Count; i++)
        {
            Vector3 transformedA = lidarToCameraTransform.MultiplyPoint3x4(pointsA[i]);
            float error = Vector3.Distance(transformedA, pointsB[i]);
            totalError += error * error;
        }
        return Mathf.Sqrt(totalError / pointsA.Count);
    }

    public Matrix4x4 GetLiDARToCameraTransform()
    {
        return lidarToCameraTransform;
    }

    public float GetCalibrationError()
    {
        return calibrationError;
    }
}
```

## Validation and Testing

### Multi-Sensor Validation Framework

Create a validation framework for multi-sensor systems:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class MultiSensorValidator : MonoBehaviour
{
    [Header("Validation Configuration")]
    [SerializeField] private MultiSensorFusion fusionSystem;
    [SerializeField] private UnityLiDAR lidarSource;
    [SerializeField] private UnityDepthCamera cameraSource;
    [SerializeField] private UnityIMU imuSource;
    [SerializeField] private Transform referenceObject;

    [Header("Validation Parameters")]
    [SerializeField] private float positionTolerance = 0.05f;
    [SerializeField] private float orientationTolerance = 2f; // degrees
    [SerializeField] private float accuracyThreshold = 0.95f;

    [Header("Validation Results")]
    [SerializeField] private float positionAccuracy = 0f;
    [SerializeField] private float orientationAccuracy = 0f;
    [SerializeField] private float overallAccuracy = 0f;

    private List<float> positionErrors = new List<float>();
    private List<float> orientationErrors = new List<float>();
    private float nextValidation = 0f;
    private float validationInterval = 2f;

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
        if (referenceObject == null || fusionSystem == null) return;

        // Get fused data
        Vector3 fusedPosition = fusionSystem.GetFusedPosition();
        Quaternion fusedOrientation = fusionSystem.GetFusedOrientation();

        // Get ground truth
        Vector3 truePosition = referenceObject.position;
        Quaternion trueOrientation = referenceObject.rotation;

        // Calculate errors
        float positionError = Vector3.Distance(fusedPosition, truePosition);
        float orientationError = Quaternion.Angle(fusedOrientation, trueOrientation);

        // Store errors
        positionErrors.Add(positionError);
        orientationErrors.Add(orientationError);

        // Calculate accuracies
        bool positionValid = positionError <= positionTolerance;
        bool orientationValid = orientationError <= orientationTolerance;

        // Update accuracy metrics
        positionAccuracy = CalculateAccuracy(positionErrors, positionTolerance);
        orientationAccuracy = CalculateAccuracy(orientationErrors, orientationTolerance);
        overallAccuracy = (positionAccuracy + orientationAccuracy) / 2f;

        // Log results
        string result = $"Validation - Pos Acc: {positionAccuracy:P2}, " +
                       $"Ori Acc: {orientationAccuracy:P2}, " +
                       $"Overall: {overallAccuracy:P2}";

        if (overallAccuracy < accuracyThreshold)
        {
            Debug.LogWarning(result);
        }
        else
        {
            Debug.Log(result);
        }

        // Keep only recent validation results
        if (positionErrors.Count > 100)
        {
            positionErrors.RemoveRange(0, 50);
            orientationErrors.RemoveRange(0, 50);
        }
    }

    float CalculateAccuracy(List<float> errors, float tolerance)
    {
        if (errors.Count == 0) return 0f;

        int validCount = 0;
        foreach (float error in errors)
        {
            if (error <= tolerance) validCount++;
        }

        return (float)validCount / errors.Count;
    }

    public float GetPositionAccuracy()
    {
        return positionAccuracy;
    }

    public float GetOrientationAccuracy()
    {
        return orientationAccuracy;
    }

    public float GetOverallAccuracy()
    {
        return overallAccuracy;
    }

    public bool IsSystemValid()
    {
        return overallAccuracy >= accuracyThreshold;
    }

    public void ResetValidation()
    {
        positionErrors.Clear();
        orientationErrors.Clear();
        positionAccuracy = 0f;
        orientationAccuracy = 0f;
        overallAccuracy = 0f;
    }
}
```

## Exercise: Multi-Sensor Integration

1. Implement sensor synchronization for LiDAR, depth camera, and IMU in both Gazebo and Unity
2. Configure proper coordinate system transforms between all sensors
3. Implement a basic sensor fusion algorithm combining data from all three sensors
4. Create a calibration procedure to align sensor coordinate systems
5. Validate the fused sensor data against known ground truth
6. Document the performance of your multi-sensor system
7. Identify and address any synchronization or calibration issues

## Best Practices for Sensor Integration

1. **Start Simple**: Begin with basic fusion and gradually add complexity
2. **Validate Continuously**: Implement validation at each stage of integration
3. **Monitor Performance**: Track computational resources and adjust accordingly
4. **Handle Failures Gracefully**: Design systems that can operate with partial sensor data
5. **Document Calibration**: Maintain detailed records of calibration procedures and results

## Troubleshooting Common Issues

### Issue: Sensor data is not synchronized
**Solution**: Implement proper timestamp-based synchronization with appropriate tolerance values.

### Issue: Coordinate systems are not aligned
**Solution**: Perform careful calibration and verify transforms between all sensors.

### Issue: Fusion algorithm is computationally expensive
**Solution**: Optimize algorithms, reduce data rates, or use more efficient data structures.

## Next Steps

After implementing sensor integration, proceed to [Validation Techniques](./validation.md) to learn how to validate the accuracy of your multi-sensor system and ensure it meets the required performance specifications.