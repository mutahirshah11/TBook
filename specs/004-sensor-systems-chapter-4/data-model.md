# Data Model: Chapter 4 Concept Map

**Purpose**: Defines the key conceptual entities and their relationships within the chapter.

## Concepts

### 1. The Sensor Types
- **LiDAR (Light Detection and Ranging)**
  - *Data*: Point Cloud (x, y, z intensity).
  - *Principle*: Time-of-Flight (ToF).
  - *Role*: Long-range geometry, obstacle avoidance.
- **Camera (RGB / RGB-D)**
  - *Data*: 2D Image (pixels) / Depth Map.
  - *Principle*: Passive light collection / Active IR projection (for depth).
  - *Role*: Semantic understanding (what is this?), detailed close-range geometry.
- **IMU (Inertial Measurement Unit)**
  - *Data*: Acceleration (linear), Angular Velocity (rotational).
  - *Principle*: MEMS vibrating structures.
  - *Role*: The "Inner Ear" - balance, gravity vector reference.
- **Force/Torque (F/T) Sensor**
  - *Data*: Force vector (Fx, Fy, Fz), Torque vector (Tx, Ty, Tz).
  - *Principle*: Strain gauges measuring micro-deflections.
  - *Role*: The "Sense of Touch" - contact detection, weight estimation.

### 2. The Fusion Concept
- **State Estimation**: The robot's internal belief of where it is and what it's doing.
- **Uncertainty**: The error margin associated with every sensor reading (Noise).
- **Fusion**: Combining noisy inputs to reduce uncertainty.
  - *Example*: IMU is fast but drifts; LiDAR is slow but accurate. Fused = Fast and Accurate.

## Relationships

- **LiDAR + Camera** -> Colored Point Clouds (Geometry + Semantics).
- **IMU + Leg Odometry** -> Stable Body Posture Estimate.
- **Arm Position + F/T Sensor** -> Safe Manipulation (stopping on contact).
