---
sidebar_position: 2
---

# The Robotic Nervous System (ROS 2)

Robot Operating System 2 (ROS 2) serves as the communication backbone for humanoid robots, providing the infrastructure needed for distributed systems to interact seamlessly. Just as the nervous system enables communication between different parts of the human body, ROS 2 enables communication between different software components and hardware systems in a robot.

## ROS 2 Architecture

ROS 2 is built on a distributed architecture that enables multiple processes to communicate with each other, whether they are running on the same machine or across a network. The architecture is based on the Data Distribution Service (DDS) standard, which provides a middleware layer for communication.

### Core Concepts

The fundamental building blocks of ROS 2 include:

- **Nodes**: Processes that perform computation. Nodes are the basic unit of execution in ROS 2.
- **Topics**: Named buses over which nodes exchange messages. Topics implement a publish-subscribe communication pattern.
- **Services**: Synchronous request-response communication pattern between nodes.
- **Actions**: Asynchronous communication pattern for long-running tasks with feedback.
- **Parameters**: Configuration values that can be set at runtime.

### Communication Patterns

ROS 2 supports several communication patterns:

1. **Publish-Subscribe (Topics)**: Many-to-many communication where publishers send messages to topics and subscribers receive messages from topics. This is ideal for streaming data like sensor readings.

2. **Request-Response (Services)**: One-to-one synchronous communication where a client sends a request and waits for a response from a server. This is suitable for operations that have a clear start and end.

3. **Action Interface**: One-to-one communication for long-running tasks that require feedback and the ability to cancel. This is perfect for navigation, manipulation, and other tasks that take time to complete.

### Quality of Service (QoS)

ROS 2 provides Quality of Service settings that allow fine-tuning of communication behavior:

- **Reliability**: Whether messages are guaranteed to be delivered
- **Durability**: Whether late-joining subscribers receive previous messages
- **History**: How many messages to store for late-joining subscribers
- **Depth**: Size of the publisher queue

## Python Agent Integration using rclpy

The `rclpy` library is the Python client library for ROS 2, enabling Python-based agents to interact with the ROS 2 ecosystem. This is particularly useful for AI agents that need to interface with robotic hardware.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent')
        # Initialize publishers, subscribers, services, etc.

    def destroy_node(self):
        # Cleanup operations
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    agent_node = RobotAgentNode()

    try:
        rclpy.spin(agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publishers and Subscribers

Creating a publisher to send messages:

```python
from std_msgs.msg import String

class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent')
        self.publisher = self.create_publisher(String, 'robot_commands', 10)
        self.timer = self.create_timer(0.5, self.publish_command)  # Publish every 0.5 seconds

    def publish_command(self):
        msg = String()
        msg.data = f'Command sent at {self.get_clock().now()}'
        self.publisher.publish(msg)
```

Creating a subscriber to receive messages:

```python
from std_msgs.msg import String

class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

### Services

Creating a service server:

```python
from example_interfaces.srv import AddTwoInts

class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

Creating a service client:

```python
from example_interfaces.srv import AddTwoInts

class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## URDF for Humanoid Robots

Unified Robot Description Format (URDF) is an XML format for representing a robot model. For humanoid robots, URDF describes the physical structure, kinematic chains, and visual/collision properties.

### Basic URDF Structure

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
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.7"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### Humanoid-Specific Considerations

Humanoid robots require special attention to:

- **Degrees of Freedom**: Ensuring sufficient joints for human-like movement
- **Kinematic Chains**: Properly modeling arms, legs, and spine for coordinated movement
- **Center of Mass**: Critical for balance and stability
- **Collision Avoidance**: Preventing self-collision during movement

## Examples of Basic Robot Control

### Simple Movement Controller

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, speed=0.5):
        msg = Twist()
        msg.linear.x = speed
        self.publisher.publish(msg)

    def turn(self, angular_speed=0.5):
        msg = Twist()
        msg.angular.z = angular_speed
        self.publisher.publish(msg)

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = MovementController()

    # Move forward for 2 seconds
    controller.move_forward(0.5)
    rclpy.spin_once(controller, timeout_sec=2.0)

    # Stop
    controller.stop()

    controller.destroy_node()
    rclpy.shutdown()
```

ROS 2 provides the essential infrastructure for building complex humanoid robots, enabling different components to communicate effectively while maintaining modularity and scalability. This forms the foundation for higher-level AI systems to control the robot's behavior.