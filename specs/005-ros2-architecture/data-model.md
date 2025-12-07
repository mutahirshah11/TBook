# Data Model (Content Outline)

## Chapter 5: ROS 2 Architecture and Core Concepts

### 1. Introduction
- Goal: Understand how ROS 2 nodes communicate and the underlying middleware.
- Prereq: Basic Python, ROS 2 installation.

### 2. The ROS 2 Graph
- **Node**: The fundamental unit.
- **Discovery**: How nodes find each other (automated via DDS).

### 3. Communication Patterns (The "Big 3")
- **Topics (Pub/Sub)**: Asynchronous streaming (e.g., sensor data).
  - *Diagram*: Pub -> Topic -> Sub
  - *Code*: Python Simple Publisher/Subscriber
- **Services (Req/Resp)**: Synchronous commands (e.g., "Reset Odometer").
  - *Diagram*: Client -> Request -> Server -> Response -> Client
- **Actions (Goal/Feedback/Result)**: Long-running tasks (e.g., "Navigate to X").
  - *Diagram*: Goal -> Server (Feedback...) -> Result

### 4. Configuration & Orchestration
- **Parameters**: Runtime configuration (integers, strings).
- **Launch Files**: Starting multiple nodes with Python.

### 5. Under the Hood: DDS & Middleware
- **RMW (Robot Middleware Interface)**: Abstraction layer.
- **DDS (Data Distribution Service)**: The transport layer.
- **QoS (Quality of Service)**: Reliability vs. Best Effort.

### 6. Advanced Concepts (Brief)
- **SROS2**: Security (AuthN/AuthZ).
- **Rosbag**: Recording data.

### 7. Summary & Checklist
