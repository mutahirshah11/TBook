# ROS2-Unity Interaction Contract

**Protocol**: TCP/IP (Custom Handshake) via `ROS-TCP-Connector`
**Port**: 10000 (default)
**Serialization**: BSON/JSON (depending on configuration, typically BSON for performance)

## Topics

### Inbound (ROS2 -> Unity)

**Topic**: `/cmd_vel`
**Type**: `geometry_msgs/Twist`
**Unity Handler**: `RosSubscriber.cs`
**Effect**: Applies velocity to the robot's base or wheels.

```json
{
  "linear": { "x": 1.0, "y": 0.0, "z": 0.0 },
  "angular": { "x": 0.0, "y": 0.0, "z": 0.5 }
}
```

**Topic**: `/color_change` (Example)
**Type**: `std_msgs/ColorRGBA`
**Unity Handler**: `ColorSubscriber.cs`
**Effect**: Changes the material color of a specific GameObject.

### Outbound (Unity -> ROS2)

**Topic**: `/joint_states`
**Type**: `sensor_msgs/JointState`
**Unity Source**: `JointStatePublisher.cs` (or URDF Robot component)
**Effect**: Publishes the current angles of the ArticulationBodies.

```json
{
  "header": { ... },
  "name": ["joint1", "joint2"],
  "position": [0.52, -1.2],
  "velocity": [0.0, 0.0],
  "effort": [0.0, 0.0]
}
```

## Service Calls (Unity -> ROS2)

**Service**: `/unity_reset` (Example)
**Type**: `std_srvs/Trigger`
**Effect**: Resets the simulation state in ROS2 (e.g., resets generic counters or algorithms).

## Handshake Process

1. **Unity (Client)** initiates TCP connection to **ROS2 (Server)** IP:10000.
2. **Unity** sends handshake header containing version info.
3. **ROS2** accepts and maintains open socket.
4. **Unity** sends "Register Subscriber" meta-message for each topic it listens to.
5. **Unity** sends "Register Publisher" meta-message for each topic it writes to.
