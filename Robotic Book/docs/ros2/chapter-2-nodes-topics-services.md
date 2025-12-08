---
sidebar_position: 2
---

# Chapter 2: ROS 2 Nodes, Topics, and Services

## Conceptual Explanation

The fundamental building blocks of ROS 2 include:

- **Nodes**: Processes that perform computation. Nodes are the basic unit of execution in ROS 2, encapsulating functionality such as sensor processing, control algorithms, or user interfaces.
- **Topics**: Named buses over which nodes exchange messages. Topics implement a publish-subscribe communication pattern that enables one-to-many communication.
- **Services**: Synchronous request-response communication pattern between nodes that provides direct interaction with guaranteed response.
- **Actions**: Asynchronous communication pattern for long-running tasks with feedback and cancellation capabilities.
- **Parameters**: Configuration values that can be set at runtime to control node behavior.

## Node Lifecycle

Nodes in ROS 2 follow a well-defined lifecycle that includes:

1. **Unconfigured State**: Initial state after node creation
2. **Inactive State**: After configuration but before activation
3. **Active State**: Fully operational with all interfaces available
4. **Finalized State**: Clean shutdown after cleanup operations

This lifecycle management enables sophisticated system management for complex humanoid robots, allowing for graceful startup, reconfiguration, and shutdown procedures.

## Topics vs Services vs Actions

ROS 2 supports three primary communication patterns:

1. **Publish-Subscribe (Topics)**: Many-to-many communication where publishers send messages to topics and subscribers receive messages from topics. This is ideal for streaming data like sensor readings, camera feeds, or joint states. The communication is asynchronous and decoupled in time.

2. **Request-Response (Services)**: One-to-one synchronous communication where a client sends a request and waits for a response from a server. This is suitable for operations that have a clear start and end, such as saving a map or executing a calibration procedure.

3. **Action Interface**: One-to-one communication for long-running tasks that require feedback and the ability to cancel. This is perfect for navigation, manipulation, and other tasks that take time to complete and may need to report progress or be interrupted.

## Real-World Humanoid Robot Examples

In humanoid robots, these communication patterns are used in specific ways:

- **Topics**: Joint positions and velocities, IMU data, camera feeds, LIDAR scans, and other sensor streams
- **Services**: Robot calibration, parameter updates, and emergency stop procedures
- **Actions**: Navigation to waypoints, manipulation tasks, and complex behaviors that require feedback