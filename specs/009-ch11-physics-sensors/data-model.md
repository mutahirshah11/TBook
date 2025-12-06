# Data Model for Chapter 11: Physics Simulation and Sensor Simulation

This chapter focuses on conceptual understanding of physics and sensor simulation rather than a data model for a software system. However, the key entities discussed are:

## Entities:

### Physics Engine
*   **Description**: The core component of Gazebo responsible for simulating realistic physical interactions between models and the environment.
*   **Attributes (conceptual)**:
    *   `gravity`: Vector defining the direction and magnitude of gravitational force.
    *   `time_step`: Simulation time step.
    *   `solver`: Physics solver parameters (e.g., iterations, tolerance).
*   **Relationships**: Governs the behavior of `Sensor`s and `SDF` models.

### Sensor
*   **Description**: A simulated device that mimics real-world sensors, generating data (e.g., images, point clouds, IMU readings) that a robot's control system would use.
*   **Attributes (conceptual)**:
    *   `type`: Type of sensor (e.g., camera, lidar, imu).
    *   `topic`: ROS 2 topic where sensor data is published.
    *   `update_rate`: Frequency at which sensor data is generated.
    *   `noise`: Parameters for simulating sensor noise.
*   **Relationships**: Configured via `SDF`, publishes data to `ROS 2 Topic`.

### SDF (Simulation Description Format)
*   **Description**: The primary XML format used in Gazebo to define physics properties of the world and models, and to configure sensors.
*   **Attributes (conceptual)**:
    *   `physics`: Global physics properties for the world.
    *   `model`: Contains physics and sensor definitions for robots/objects.
    *   `sensor`: Sensor configurations within models.
*   **Relationships**: Used by `Physics Engine` and to define `Sensor`s.

### ROS 2 Topic
*   **Description**: A named bus over which nodes exchange messages, used here to transmit simulated sensor data from Gazebo to the ROS 2 ecosystem.
*   **Attributes (conceptual)**:
    *   `name`: Unique name of the topic.
    *   `type`: Message type transmitted over the topic.
*   **Relationships**: `Sensor`s publish data to `ROS 2 Topic`s.
