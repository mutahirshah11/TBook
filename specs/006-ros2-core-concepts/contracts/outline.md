# Chapter Outlines

## Chapter 6: Nodes, Topics, Services, and Actions

### 1. Introduction to the ROS Graph
- Concept: The "Graph" abstraction.
- Entity: **Nodes** (Processes that perform computation).
- Command: `ros2 node list`, `ros2 node info`.

### 2. Topics: Asynchronous Streaming
- Concept: Publisher / Subscriber model (Many-to-Many).
- Diagram: Node A -> Topic -> Node B.
- Code Example:
    - `MinimalPublisher` class.
    - `MinimalSubscriber` class.
- Command: `ros2 topic list`, `ros2 topic echo`, `ros2 topic pub`.

### 3. Services: Synchronous Request/Response
- Concept: Client / Server model (One-to-One).
- Use case: "Calculate this", "Turn on LED".
- Diagram: Client -> Request -> Server -> Response -> Client.
- Code Example:
    - `MinimalService` (Server).
    - `MinimalClient` (Client).
- Command: `ros2 service list`, `ros2 service call`.

### 4. Actions: Long-Running Goals
- Concept: Goal / Feedback / Result model.
- Use case: "Navigation", "Robotic Arm Movement".
- Distinction from Services: Asynchronous, cancellable, provides feedback.
- Diagram: Client -> Goal -> Server -> Feedback -> Client -> Result.
- Command: `ros2 action list`, `ros2 action send_goal`.

## Chapter 7: Building ROS 2 Packages with Python

### 1. The ROS 2 Workspace
- Structure: `src`, `build`, `install`, `log`.
- Best practice: Always build in root, source `install/setup.bash`.

### 2. Creating a Python Package
- Command: `ros2 pkg create --build-type ament_python <pkg_name>`.
- File walkthrough: `package.xml`, `setup.py`, `setup.cfg`.

### 3. Managing Dependencies
- `package.xml`: `<exec_depend>`, `<test_depend>`.
- Key dependency: `rclpy`, `std_msgs`.

### 4. Defining Entry Points
- `setup.py`: `entry_points={'console_scripts': [...]}`.
- Mapping executables to Python functions.

### 5. Building and Running
- Tool: `colcon build --symlink-install`.
- Sourcing: `source install/setup.bash`.
- Execution: `ros2 run <pkg_name> <executable_name>`.
