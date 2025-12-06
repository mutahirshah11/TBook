# Data Model: ROS 2 Core Concepts

## Documentation Entities (Docusaurus)

### File Structure
The feature will introduce a new directory `docs/part2` containing the following Markdown files:

1.  **`docs/part2/chapter-6-nodes-topics-services-actions.md`**
    *   **Title**: Nodes, Topics, Services, and Actions
    *   **Slug**: `/part2/ros2-core-concepts`
    *   **Key Sections**:
        *   Introduction to Nodes
        *   Topics (Pub/Sub)
        *   Services (Client/Server)
        *   Actions (Goal/Feedback/Result)
        *   Introspection Tools (`ros2` CLI)

2.  **`docs/part2/chapter-7-building-packages.md`**
    *   **Title**: Building ROS 2 Packages with Python
    *   **Slug**: `/part2/building-packages`
    *   **Key Sections**:
        *   Workspaces and Overlays
        *   Creating a Package (`ros2 pkg create`)
        *   `package.xml` and `setup.py`
        *   Building with `colcon`
        *   Sourcing and Running

## Code Entities (Reader's Workspace)

The content will guide the reader to create the following structure in their local environment:

```text
~/ros2_ws/
└── src/
    └── my_first_package/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── my_first_package
        └── my_first_package/
            ├── __init__.py
            ├── simple_publisher.py
            ├── simple_subscriber.py
            ├── simple_service_server.py
            └── simple_service_client.py
```

### Code Artifacts

#### 1. `simple_publisher.py`
- **Class**: `SimplePublisher(Node)`
- **Methods**:
    - `__init__`: Init node, create publisher, create timer.
    - `timer_callback`: Publish message, log output.

#### 2. `simple_subscriber.py`
- **Class**: `SimpleSubscriber(Node)`
- **Methods**:
    - `__init__`: Init node, create subscription.
    - `listener_callback`: Log received message.

#### 3. `simple_service_server.py`
- **Class**: `SimpleServiceServer(Node)`
- **Methods**:
    - `__init__`: Init node, create service.
    - `service_callback`: Receive request, compute response, return response.

#### 4. `simple_service_client.py`
- **Class**: `SimpleServiceClient(Node)`
- **Methods**:
    - `__init__`: Init node, create client.
    - `send_request`: Send async request, await future.
