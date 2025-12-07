---
title: "Nodes, Topics, Services, and Actions"
sidebar_label: "Chapter 6: Nodes, Topics, Services, and Actions"
---

# Chapter 6: Nodes, Topics, Services, and Actions

This chapter explores the fundamental communication primitives in ROS 2. We'll cover Nodes, the ROS Graph, and the three main communication patterns: Topics (Publisher/Subscriber), Services (Client/Server), and Actions (Goal/Feedback/Result).

## 1. Introduction to the ROS Graph

The **ROS Graph** is a network of ROS 2 elements processing data at the same time. It encompasses all executables and the connections between them if you could map them all out.

### Nodes

A **Node** is the fundamental unit of computation in ROS 2. Each node in a ROS 2 system is responsible for a specific task (e.g., controlling wheel motors, reading laser data, or path planning). By breaking a complex robotic system into many small, modular nodes, the system becomes more robust and easier to debug.

#### Key Characteristics:
*   **Modularity**: A robot system is comprised of many nodes working together.
*   **Single Purpose**: Ideally, a node should perform one specific function.
*   **Discovery**: Nodes automatically discover each other on the same network domain.

#### Lifecycle:
Nodes have a lifecycle (Unconfigured, Inactive, Active, Finalized) which allows for greater control over the state of the system, though for basic applications, we often just consider them "running" or "stopped".

#### Inspection:
You can see the currently running nodes using the CLI:

```bash
ros2 node list
```

And get details about a specific node:

```bash
ros2 node info <node_name>
```

## 2. Topics: Asynchronous Streaming

**Topics** implement a **Publisher / Subscriber** model. This is the most common way to move data between nodes. It is a "many-to-many" connection: a node may publish data to any number of topics and have subscriptions to any number of topics.

*   **Asynchronous**: The publisher sends data without waiting for a reply.
*   **Decoupled**: Publishers and subscribers don't need to know about each other's existence.

![Topic Diagram](https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif)
*(Source: ROS 2 Documentation)*

### Python Example: Simple Publisher

:::tip Try it yourself
To run this code, you'll need to create a ROS 2 package first. See **[Chapter 7: Building ROS 2 Packages with Python](chapter-7-building-packages.md)** for step-by-step instructions.
:::

Below is a complete, runnable example of a minimal publisher using `rclpy`.

```python title="simple_publisher.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        # Create a publisher on the 'topic' topic, with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        # Create a timer to call timer_callback every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    
    # Destroy the node explicitly
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Example: Simple Subscriber

Here is the corresponding subscriber.

```python title="simple_subscriber.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        # Create a subscription to the 'topic' topic
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### CLI Tools for Topics

*   List active topics: `ros2 topic list`
*   See data on a topic: `ros2 topic echo <topic_name>`
*   Publish data manually: `ros2 topic pub <topic_name> <msg_type> "<args>"`

## 3. Services: Synchronous Request/Response

**Services** implement a **Client / Server** model. This pattern is used for "remote procedure calls" or tasks that require a reply.

*   **Synchronous**: The client sends a request and waits for the server to respond.
*   **One-to-One**: A service is typically hosted by one server node and called by client nodes.

![Service Diagram](https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif)
*(Source: ROS 2 Documentation)*

### Python Example: Simple Service Server

This node provides a service that adds two integers. Note: This example uses `example_interfaces.srv.AddTwoInts`.

```python title="simple_service_server.py"
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class SimpleServiceServer(Node):

    def __init__(self):
        super().__init__('simple_service_server')
        # Create a service named 'add_two_ints'
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Example: Simple Service Client

This node calls the service.

```python title="simple_service_client.py"
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class SimpleServiceClient(Node):

    def __init__(self):
        super().__init__('simple_service_client')
        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Call service asynchronously
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    simple_service_client = SimpleServiceClient()
    response = simple_service_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    simple_service_client.get_logger().info(
        'Result of add_two_ints: %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### CLI Tools for Services

*   List active services: `ros2 service list`
*   Call a service manually: `ros2 service call <service_name> <srv_type> "<args>"`

## 4. Actions: Long-Running Goals

**Actions** are intended for long-running tasks. They are built on top of topics and services.

*   **Goal**: The client initiates a task (e.g., "move to location X").
*   **Feedback**: The server provides periodic updates (e.g., "distance remaining: 2m").
*   **Result**: The server notifies the client upon completion (e.g., "Goal reached").
*   **Cancellable**: The client can cancel the goal mid-execution.

![Action Diagram](https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif)
*(Source: ROS 2 Documentation)*

Unlike a Service, which blocks until a response is received, an Action client is non-blocking and can receive feedback while the robot is working.

### CLI Tools for Actions

*   List active actions: `ros2 action list`
*   Send an action goal: `ros2 action send_goal <action_name> <action_type> "<goal>"`



