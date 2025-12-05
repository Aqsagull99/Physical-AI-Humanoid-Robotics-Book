---
sidebar_position: 3
---

# The Digital Twin (Gazebo & Unity)

A digital twin is a virtual replica of a physical system that enables simulation, testing, and optimization before deployment to real hardware. In the context of humanoid robotics, digital twins provide a safe and cost-effective environment to develop and test complex behaviors before running them on expensive physical robots.

## Physics Simulation

Physics simulation is the cornerstone of any effective digital twin. It accurately models the physical properties and interactions of objects in the virtual environment, including gravity, friction, collisions, and material properties.

### Key Physics Concepts

- **Rigid Body Dynamics**: The simulation of solid objects that do not deform under applied forces. This is the primary approach for simulating robot links and environmental objects.

- **Collision Detection**: The computational method for determining when two or more bodies come into contact. This is essential for realistic interaction between the robot and its environment.

- **Contact Physics**: The modeling of forces that occur when objects touch, including friction, restitution (bounciness), and surface properties.

### Gazebo Physics Engine

Gazebo, which uses the ODE (Open Dynamics Engine), DART (Dynamic Animation and Robotics Toolkit), or Bullet physics engines, provides:

- **Accurate Dynamics**: Realistic simulation of forces, torques, and motion
- **Real-time Performance**: Fast simulation suitable for control algorithm testing
- **SDF Format**: Simulation Description Format for defining models and worlds
- **ROS Integration**: Seamless integration with ROS 2 for robotics applications

Example physics configuration in SDF:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

### Unity Physics Engine

Unity's physics engine, based on the PhysX engine from NVIDIA, provides:

- **High Fidelity**: Advanced physics simulation with support for complex interactions
- **Visual Quality**: Excellent rendering capabilities for realistic visualization
- **Scripting Integration**: Direct access to physics properties through C# scripts
- **XR Support**: Native support for virtual and augmented reality applications

## Gravity and Collisions

### Gravity Configuration

Gravity is a fundamental force that affects all objects in the simulation. Properly configuring gravity is essential for realistic behavior:

In Gazebo:
```xml
<world name="default">
  <gravity>0 0 -9.8</gravity>
  <!-- Other world properties -->
</world>
```

In Unity, gravity is configured in the Physics Manager or through scripting:
```csharp
Physics.gravity = new Vector3(0, -9.81f, 0);
```

### Collision Detection

Collision detection systems must balance accuracy with performance:

- **Broad Phase**: Fast, approximate collision detection to eliminate impossible collisions
- **Narrow Phase**: Precise collision detection between potentially colliding objects
- **Continuous Collision Detection (CCD)**: Advanced method to prevent fast-moving objects from passing through each other

### Collision Response

When collisions occur, the physics engine calculates the appropriate response:

- **Elastic Collisions**: Objects bounce with conservation of momentum
- **Inelastic Collisions**: Objects may stick together or lose energy
- **Friction**: Resistance to sliding motion between surfaces
- **Restitution**: The "bounciness" of collisions

## Sensor Simulation

Sensor simulation is critical for bridging the gap between simulation and reality. It allows robots to perceive their virtual environment in ways that closely match real sensors.

### LiDAR Simulation

LiDAR sensors provide 2D or 3D point cloud data by measuring distances using laser light. In simulation:

**Gazebo LiDAR Plugin:**
```xml
<sensor name="lidar_sensor" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
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
      <namespace>/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### Depth Camera Simulation

Depth cameras provide both color images and depth information for each pixel:

**Gazebo Depth Camera:**
```xml
<sensor name="depth_camera" type="depth">
  <camera name="depth_cam">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <ros>
      <namespace>/camera</namespace>
    </ros>
    <update_rate>30.0</update_rate>
    <depth_image_topic_name>depth/image_raw</depth_image_topic_name>
    <point_cloud_topic_name>depth/points</point_cloud_topic_name>
  </plugin>
</sensor>
```

### IMU Simulation

Inertial Measurement Units (IMUs) provide acceleration and angular velocity measurements:

**Gazebo IMU Sensor:**
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## High-Fidelity Rendering and Human-Robot Interaction in Unity

Unity excels at creating photorealistic environments that can be used for both simulation and visualization of human-robot interaction scenarios.

### Unity Robotics Simulation

Unity provides the Unity Robotics Hub and Simulation Framework that enable:

- **ROS Integration**: Direct communication between Unity and ROS 2 using the ROS TCP Connector
- **High-Fidelity Graphics**: State-of-the-art rendering for realistic sensor simulation
- **XR Development**: Support for virtual and augmented reality applications
- **Physics Simulation**: Accurate PhysX physics engine for robot dynamics

### Setting up Unity for Robotics

To integrate Unity with ROS 2:

1. **Install Unity Robotics Package**: Use the Unity Package Manager to install the ROS TCP Connector
2. **Configure ROS Communication**: Set up publishers and subscribers to communicate with ROS nodes
3. **Create Robot Models**: Import URDF files or create robot models directly in Unity
4. **Implement Control Systems**: Develop controllers that can operate both in simulation and on real robots

### Human-Robot Interaction Simulation

Unity's capabilities for simulating human-robot interaction include:

- **Character Animation**: Realistic human avatars with natural movement
- **Social Interaction**: Simulating social behaviors and responses
- **Environmental Context**: Creating realistic indoor and outdoor environments
- **Multi-Modal Perception**: Simulating how robots perceive humans through vision, audio, and other sensors

Example Unity script for ROS communication:

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string robotTopic = "/unity_robot_control";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.instance;
    }

    // Update is called once per frame
    void Update()
    {
        // Create a message
        Vector3Msg robotPosition = new Vector3Msg(transform.position.x, transform.position.y, transform.position.z);

        // Publish the message
        ros.Publish(robotTopic, robotPosition);
    }
}
```

Digital twins using both Gazebo and Unity provide comprehensive simulation capabilities for humanoid robots, enabling development and testing in safe, controlled virtual environments before deployment to physical hardware.