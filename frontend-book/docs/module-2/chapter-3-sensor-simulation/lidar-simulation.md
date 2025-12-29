# LiDAR Simulation Tutorial

## Overview

This tutorial covers the implementation of LiDAR sensors in digital twin environments using both Gazebo and Unity. You'll learn to create realistic LiDAR simulation that generates accurate point cloud data for robotics applications.

## Prerequisites

- Understanding of sensor simulation fundamentals
- Basic knowledge of Gazebo and Unity
- Completion of previous chapters

## LiDAR Sensor Theory

### How LiDAR Works

LiDAR sensors emit laser pulses and measure the time it takes for the light to return after reflecting off objects. This creates precise 3D point cloud data that represents the environment.

**Key Parameters:**
- **Range**: Maximum distance the sensor can detect (typically 10-100m)
- **Resolution**: Angular resolution of the sensor (horizontal and vertical)
- **Field of View**: Angular coverage of the sensor
- **Update Rate**: How frequently the sensor produces new data
- **Accuracy**: Precision of distance measurements

## Implementing LiDAR in Gazebo

### Step 1: Adding LiDAR to URDF Model

First, add a LiDAR sensor to your robot model in URDF:

```xml
<!-- Add to your robot URDF -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
</joint>

<!-- LiDAR sensor definition -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>   <!-- 180 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### Step 2: Configuring Advanced LiDAR Parameters

For more realistic LiDAR simulation, configure additional parameters:

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1440</samples>  <!-- Higher resolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>1</samples>  <!-- For 2D LiDAR -->
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>0</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <!-- Noise model for realistic behavior -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### Step 3: 3D LiDAR Configuration

For a 3D LiDAR like Velodyne, use multiple vertical beams:

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="velodyne_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>32</samples>  <!-- 32 beams for VLP-32C -->
          <resolution>1</resolution>
          <min_angle>-0.5236</min_angle>  <!-- -30 degrees -->
          <max_angle>0.15708</max_angle>  <!-- 9 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.3</min>
        <max>100.0</max>
        <resolution>0.001</resolution>
      </range>
    </ray>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.008</stddev>
    </noise>
    <plugin name="velodyne_controller" filename="libgazebo_ros_velodyne_gpu_laser.so">
      <ros>
        <namespace>velodyne</namespace>
        <remapping>~/out:=pointcloud</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Implementing LiDAR in Unity

### Step 1: Creating a LiDAR Simulation Script

Create a C# script to simulate LiDAR functionality in Unity:

```csharp
using UnityEngine;
using System.Collections.Generic;
using System;

public class UnityLiDAR : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    [SerializeField] private float range = 30.0f;
    [SerializeField] private float horizontalFOV = 360f;
    [SerializeField] private int horizontalResolution = 720;
    [SerializeField] private float verticalFOV = 10f;
    [SerializeField] private int verticalResolution = 1;
    [SerializeField] private float updateRate = 10f;
    [SerializeField] private LayerMask detectionLayers = -1;

    [Header("Noise Parameters")]
    [SerializeField] private float noiseStdDev = 0.01f;
    [SerializeField] private bool visualizeRays = false;

    private float nextUpdate = 0f;
    private List<Vector3> pointCloud = new List<Vector3>();
    private List<Color> pointColors = new List<Color>();

    void Update()
    {
        if (Time.time >= nextUpdate)
        {
            SimulateLiDAR();
            nextUpdate = Time.time + (1f / updateRate);
        }
    }

    void SimulateLiDAR()
    {
        pointCloud.Clear();
        pointColors.Clear();

        float horizontalStep = horizontalFOV / horizontalResolution;
        float verticalStep = verticalResolution > 1 ? verticalFOV / verticalResolution : 0;

        for (int v = 0; v < verticalResolution; v++)
        {
            float vAngle = -verticalFOV / 2f + v * verticalStep;

            for (int h = 0; h < horizontalResolution; h++)
            {
                float hAngle = -horizontalFOV / 2f + h * horizontalStep;

                // Calculate ray direction
                Vector3 direction = CalculateRayDirection(hAngle, vAngle);

                // Perform raycast
                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, range, detectionLayers))
                {
                    // Add noise to distance
                    float noisyDistance = hit.distance + GetNoise();

                    // Calculate actual hit point with noise
                    Vector3 noisyPoint = transform.position + direction.normalized * noisyDistance;

                    pointCloud.Add(noisyPoint);
                    pointColors.Add(GetColorForDistance(hit.distance));
                }
                else
                {
                    // Add point at maximum range if no hit
                    Vector3 maxPoint = transform.position + direction.normalized * range;
                    pointCloud.Add(maxPoint);
                    pointColors.Add(Color.gray);
                }

                // Optional: visualize rays for debugging
                if (visualizeRays)
                {
                    Debug.DrawRay(transform.position, direction * range, Color.red, 0.1f);
                }
            }
        }
    }

    Vector3 CalculateRayDirection(float horizontalAngle, float verticalAngle)
    {
        // Convert angles to radians
        float hRad = horizontalAngle * Mathf.Deg2Rad;
        float vRad = verticalAngle * Mathf.Deg2Rad;

        // Calculate direction vector
        Vector3 direction = transform.forward;
        direction = Quaternion.Euler(0, hRad, 0) * direction;
        direction = Quaternion.Euler(-vRad, 0, 0) * direction;

        return direction;
    }

    float GetNoise()
    {
        // Generate Gaussian noise using Box-Muller transform
        float u1 = UnityEngine.Random.value;
        float u2 = UnityEngine.Random.value;
        if (u1 < float.Epsilon) u1 = 0.01f; // Avoid log(0)
        float normal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
        return normal * noiseStdDev;
    }

    Color GetColorForDistance(float distance)
    {
        // Map distance to color (blue for close, red for far)
        float normalizedDistance = Mathf.Clamp01(distance / range);
        return Color.Lerp(Color.blue, Color.red, normalizedDistance);
    }

    public List<Vector3> GetPointCloud()
    {
        return new List<Vector3>(pointCloud);
    }

    public void RenderPointCloud(MeshRenderer pointCloudRenderer, Material pointMaterial)
    {
        if (pointCloud.Count == 0) return;

        // Create a mesh to represent the point cloud
        Mesh pointMesh = new Mesh();
        Vector3[] vertices = pointCloud.ToArray();
        int[] triangles = new int[pointCloud.Count * 3];

        // Create triangles for each point (or use a different visualization method)
        for (int i = 0; i < pointCloud.Count; i++)
        {
            // This is a simplified approach - for real point cloud visualization
            // you might want to use a point cloud renderer or compute shader
        }

        pointMesh.vertices = vertices;
        pointMesh.colors = pointColors.ToArray();
        pointMesh.RecalculateBounds();
    }
}
```

### Step 2: Creating a Point Cloud Visualization

Create a script to visualize the point cloud data:

```csharp
using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class PointCloudVisualizer : MonoBehaviour
{
    [SerializeField] private UnityLiDAR lidarSource;
    [SerializeField] private Material pointMaterial;
    [SerializeField] private float pointSize = 0.05f;

    private Mesh pointCloudMesh;
    private List<Vector3> pointPositions = new List<Vector3>();
    private List<Color> pointColors = new List<Color>();

    void Start()
    {
        pointCloudMesh = new Mesh();
        GetComponent<MeshFilter>().mesh = pointCloudMesh;
    }

    void Update()
    {
        if (lidarSource != null)
        {
            // Get the latest point cloud data
            List<Vector3> newPoints = lidarSource.GetPointCloud();

            if (newPoints.Count > 0)
            {
                UpdatePointCloudMesh(newPoints);
            }
        }
    }

    void UpdatePointCloudMesh(List<Vector3> newPoints)
    {
        pointPositions.Clear();
        pointColors.Clear();

        // Transform points to local space
        foreach (Vector3 worldPoint in newPoints)
        {
            Vector3 localPoint = transform.InverseTransformPoint(worldPoint);
            pointPositions.Add(localPoint);
            // Use a simple color for now - you could use actual color data
            pointColors.Add(Color.white);
        }

        // Create the mesh
        CreatePointCloudMesh();
    }

    void CreatePointCloudMesh()
    {
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();
        List<Color> colors = new List<Color>();

        // Create small quads for each point
        for (int i = 0; i < pointPositions.Count; i++)
        {
            Vector3 center = pointPositions[i];
            Color color = pointColors[i];

            // Create a small quad (4 vertices) for each point
            int startIndex = vertices.Count;

            // Define the four corners of the quad
            Vector3[] quadVertices = new Vector3[4];
            quadVertices[0] = center + new Vector3(-pointSize / 2, -pointSize / 2, 0);
            quadVertices[1] = center + new Vector3(pointSize / 2, -pointSize / 2, 0);
            quadVertices[2] = center + new Vector3(-pointSize / 2, pointSize / 2, 0);
            quadVertices[3] = center + new Vector3(pointSize / 2, pointSize / 2, 0);

            // Add vertices and colors
            for (int j = 0; j < 4; j++)
            {
                vertices.Add(quadVertices[j]);
                colors.Add(color);
            }

            // Create two triangles for the quad
            triangles.Add(startIndex);
            triangles.Add(startIndex + 1);
            triangles.Add(startIndex + 2);

            triangles.Add(startIndex + 1);
            triangles.Add(startIndex + 3);
            triangles.Add(startIndex + 2);
        }

        // Update the mesh
        pointCloudMesh.Clear();
        pointCloudMesh.vertices = vertices.ToArray();
        pointCloudMesh.triangles = triangles.ToArray();
        pointCloudMesh.colors = colors.ToArray();
        pointCloudMesh.RecalculateNormals();
        pointCloudMesh.RecalculateBounds();
    }
}
```

## Processing LiDAR Data

### Step 1: LiDAR Data Processing Script

Create a script to process LiDAR data for robotics applications:

```csharp
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class LiDARProcessor : MonoBehaviour
{
    [Header("Processing Parameters")]
    [SerializeField] private UnityLiDAR lidarSource;
    [SerializeField] private float groundThreshold = 0.1f;
    [SerializeField] private float obstacleThreshold = 1.0f;
    [SerializeField] private float minClusterSize = 5f;

    private List<Vector3> currentPointCloud = new List<Vector3>();
    private List<Vector3> groundPoints = new List<Vector3>();
    private List<Vector3> obstaclePoints = new List<Vector3>();

    void Update()
    {
        if (lidarSource != null)
        {
            currentPointCloud = lidarSource.GetPointCloud();
            ProcessPointCloud();
        }
    }

    void ProcessPointCloud()
    {
        groundPoints.Clear();
        obstaclePoints.Clear();

        // Classify points as ground or obstacles
        foreach (Vector3 point in currentPointCloud)
        {
            // Simple ground classification based on Z height
            if (point.y < groundThreshold)
            {
                groundPoints.Add(point);
            }
            else if (point.y > obstacleThreshold)
            {
                obstaclePoints.Add(point);
            }
        }

        // Perform clustering to identify objects
        List<List<Vector3>> clusters = PerformClustering(obstaclePoints);

        // Further processing can be done here
        ProcessClusters(clusters);
    }

    List<List<Vector3>> PerformClustering(List<Vector3> points)
    {
        List<List<Vector3>> clusters = new List<List<Vector3>>();
        List<bool> visited = new List<bool>(new bool[points.Count]);

        for (int i = 0; i < points.Count; i++)
        {
            if (!visited[i])
            {
                List<Vector3> cluster = new List<Vector3>();
                ClusterPoints(points, i, visited, cluster);

                if (cluster.Count >= minClusterSize)
                {
                    clusters.Add(cluster);
                }
            }
        }

        return clusters;
    }

    void ClusterPoints(List<Vector3> points, int index, List<bool> visited, List<Vector3> cluster)
    {
        visited[index] = true;
        cluster.Add(points[index]);

        // Find neighboring points within a distance threshold
        for (int i = 0; i < points.Count; i++)
        {
            if (!visited[i])
            {
                float distance = Vector3.Distance(points[index], points[i]);
                if (distance < 0.5f) // Distance threshold for clustering
                {
                    ClusterPoints(points, i, visited, cluster);
                }
            }
        }
    }

    void ProcessClusters(List<List<Vector3>> clusters)
    {
        // Process each cluster (e.g., find bounding boxes, centroids, etc.)
        foreach (List<Vector3> cluster in clusters)
        {
            Vector3 centroid = CalculateCentroid(cluster);
            Vector3 bounds = CalculateBounds(cluster);

            // You could publish this information to other systems here
            Debug.Log($"Cluster found at {centroid}, bounds: {bounds}");
        }
    }

    Vector3 CalculateCentroid(List<Vector3> points)
    {
        Vector3 sum = Vector3.zero;
        foreach (Vector3 point in points)
        {
            sum += point;
        }
        return sum / points.Count;
    }

    Vector3 CalculateBounds(List<Vector3> points)
    {
        if (points.Count == 0) return Vector3.zero;

        Vector3 min = points[0];
        Vector3 max = points[0];

        foreach (Vector3 point in points)
        {
            min = Vector3.Min(min, point);
            max = Vector3.Max(max, point);
        }

        return max - min;
    }

    public List<Vector3> GetGroundPoints()
    {
        return new List<Vector3>(groundPoints);
    }

    public List<Vector3> GetObstaclePoints()
    {
        return new List<Vector3>(obstaclePoints);
    }
}
```

## Validation and Testing

### Step 1: LiDAR Data Validation

Create a validation script to ensure your LiDAR simulation is accurate:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class LiDARValidator : MonoBehaviour
{
    [Header("Validation Parameters")]
    [SerializeField] private UnityLiDAR lidarSource;
    [SerializeField] private Transform referenceObject;
    [SerializeField] private float tolerance = 0.05f;
    [SerializeField] private float accuracyThreshold = 0.95f;

    [Header("Validation Results")]
    [SerializeField] private float currentAccuracy = 0f;
    [SerializeField] private int totalPoints = 0;
    [SerializeField] private int validPoints = 0;

    private List<Vector3> groundTruthPoints = new List<Vector3>();
    private List<Vector3> simulatedPoints = new List<Vector3>();

    void Update()
    {
        if (lidarSource != null && referenceObject != null)
        {
            ValidateLiDARData();
        }
    }

    void ValidateLiDARData()
    {
        simulatedPoints = lidarSource.GetPointCloud();

        // Calculate ground truth points based on reference object
        CalculateGroundTruth();

        // Compare simulated data with ground truth
        ValidatePoints();

        // Update accuracy metrics
        UpdateAccuracyMetrics();
    }

    void CalculateGroundTruth()
    {
        groundTruthPoints.Clear();

        // For this example, we'll calculate ground truth based on a known object
        // In practice, this would come from precise measurements or CAD models
        if (referenceObject != null)
        {
            // Calculate expected points based on the reference object's geometry
            Bounds bounds = new Bounds(referenceObject.position, referenceObject.localScale);

            // Sample points on the surface of the reference object
            // This is a simplified approach - real validation would be more complex
            for (int i = 0; i < 100; i++)
            {
                Vector3 randomPoint = bounds.center + new Vector3(
                    Random.Range(-bounds.extents.x, bounds.extents.x),
                    Random.Range(-bounds.extents.y, bounds.extents.y),
                    Random.Range(-bounds.extents.z, bounds.extents.z)
                );

                groundTruthPoints.Add(randomPoint);
            }
        }
    }

    void ValidatePoints()
    {
        validPoints = 0;
        totalPoints = Mathf.Min(simulatedPoints.Count, groundTruthPoints.Count);

        for (int i = 0; i < totalPoints; i++)
        {
            float distance = Vector3.Distance(simulatedPoints[i], groundTruthPoints[i]);
            if (distance <= tolerance)
            {
                validPoints++;
            }
        }
    }

    void UpdateAccuracyMetrics()
    {
        if (totalPoints > 0)
        {
            currentAccuracy = (float)validPoints / totalPoints;
        }
        else
        {
            currentAccuracy = 0f;
        }

        // Log validation results if accuracy is below threshold
        if (currentAccuracy < accuracyThreshold)
        {
            Debug.LogWarning($"LiDAR accuracy below threshold: {currentAccuracy:P2} (threshold: {accuracyThreshold:P2})");
        }
        else
        {
            Debug.Log($"LiDAR accuracy: {currentAccuracy:P2}");
        }
    }

    public float GetAccuracy()
    {
        return currentAccuracy;
    }

    public bool IsAccuracyValid()
    {
        return currentAccuracy >= accuracyThreshold;
    }
}
```

## Exercise: LiDAR Implementation

1. Implement a LiDAR sensor on your humanoid robot model in both Gazebo and Unity
2. Configure the LiDAR with realistic parameters (range, resolution, update rate)
3. Add noise models to make the simulation more realistic
4. Create a point cloud visualization system
5. Implement basic point cloud processing (ground plane detection, obstacle detection)
6. Validate your LiDAR simulation against known geometric objects
7. Document the accuracy of your simulation and identify areas for improvement

## Troubleshooting Common Issues

### Issue: LiDAR not detecting objects
**Solution**: Check that the detection layers are properly set and that objects have colliders.

### Issue: Poor performance with high-resolution LiDAR
**Solution**: Reduce resolution or update rate, or implement more efficient raycasting algorithms.

### Issue: Point cloud visualization is slow
**Solution**: Use GPU-based point cloud rendering or reduce the number of points visualized.

## Next Steps

After implementing LiDAR simulation, proceed to [Depth Camera Simulation Tutorial](./depth-camera-simulation.md) to learn how to simulate depth cameras in your digital twin environment.