# Data Model for Chapter 10: URDF and SDF Robot Description Formats

This chapter focuses on conceptual understanding of robot description formats rather than a data model for a software system. However, the key entities discussed are:

## Entities:

### URDF (Unified Robot Description Format)
*   **Description**: An XML format used in ROS to describe a robot's kinematic and dynamic structure, visual appearance, and collision properties.
*   **Attributes (conceptual)**:
    *   `links`: Collection of rigid bodies.
    *   `joints`: Collection of connections between links.
    *   `materials`: Visual properties.
*   **Relationships**: Can be converted to `SDF`, visualized in `RViz`.

### SDF (Simulation Description Format)
*   **Description**: An XML format primarily used by Gazebo to describe robots, environments, sensors, and plugins for simulation.
*   **Attributes (conceptual)**:
    *   `models`: Collection of robots and objects.
    *   `worlds`: Defines the entire simulation environment.
    *   `sensors`: Sensor definitions.
    *   `plugins`: Simulation plugins.
*   **Relationships**: Can be generated from `URDF`, used by `Gazebo`.

### Link
*   **Description**: A rigid body part of a robot, characterized by its geometry, mass, and inertia.
*   **Attributes (conceptual)**:
    *   `name`: Unique identifier.
    *   `geometry`: Shape (box, cylinder, mesh).
    *   `mass`: Mass property.
    *   `inertia`: Inertial properties.
*   **Relationships**: Connected by `Joint`s.

### Joint
*   **Description**: A connection between two `Link`s, defining their kinematic relationship and degrees of freedom.
*   **Attributes (conceptual)**:
    *   `name`: Unique identifier.
    *   `type`: Type of joint (e.g., revolute, continuous, fixed, prismatic).
    *   `parent`: Parent `Link`.
    *   `child`: Child `Link`.
    *   `axis`: Axis of rotation/translation.
*   **Relationships**: Connects `Link`s.

### Gazebo
*   **Description**: An open-source 3D robot simulator.
*   **Attributes (conceptual)**:
    *   `version`: Specific version (e.g., Garden).
*   **Relationships**: Utilizes `SDF` for robot and world descriptions.

### RViz
*   **Description**: A 3D visualization tool in ROS.
*   **Attributes (conceptual)**:
    *   `version`: Specific ROS 2 distribution version.
*   **Relationships**: Visualizes robot models defined in `URDF`.
