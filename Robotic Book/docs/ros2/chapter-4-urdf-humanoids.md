---
sidebar_position: 4
---

# Chapter 4: Understanding URDF for Humanoids

## What URDF is and Why It Matters

Unified Robot Description Format (URDF) is an XML format for representing a robot model. For humanoid robots, URDF describes the physical structure, kinematic chains, and visual/collision properties. URDF is essential because it provides:

- **Kinematic Models**: Information needed for forward and inverse kinematics calculations
- **Collision Detection**: Geometry data for preventing self-collision and environment collision
- **Visualization**: 3D models for simulation and debugging
- **Dynamics Simulation**: Mass and inertia properties for physics simulation

## Links and Joints (Humanoid-Specific)

In humanoid robots, the link and joint structure is particularly important for mimicking human movement:

- **Links**: Represent rigid bodies such as torso, head, upper arm, lower arm, hand, thigh, shank, and foot
- **Joints**: Connect links and define the degrees of freedom between them

Humanoid-specific joint configurations include:

- **Revolute Joints**: For rotating joints like elbows and knees
- **Continuous Joints**: For joints that can rotate indefinitely like shoulders
- **Fixed Joints**: For permanent connections like attaching sensors to links
- **Prismatic Joints**: For linear motion joints (less common in humanoids)

## Sensors and Actuators Representation

URDF also represents sensors and actuators attached to the robot:

- **Gazebo Plugins**: Extensions that define how sensors and actuators behave in simulation
- **Transmission Elements**: Define how actuators connect to joints
- **Material Properties**: Define visual appearance for rendering

## Conceptual Understanding over Heavy Syntax

While URDF syntax can be complex, the key concepts for humanoid robots are:

- **Kinematic Chains**: How links connect to form arms, legs, and spine
- **Degrees of Freedom**: Ensuring sufficient joints for human-like movement
- **Center of Mass**: Critical for balance and stability
- **Collision Avoidance**: Preventing self-collision during movement

## Basic URDF Structure for Humanoids

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <cylinder length="0.8" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```