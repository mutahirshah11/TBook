# Research: Chapter 4 - Sensor Systems

**Feature**: Sensor Systems: LiDAR, Cameras, IMUs, Force/Torque Sensors
**Status**: Complete

## Technical Context & Decisions

### 1. Sensor Selection Scope
**Decision**: Limit scope to the "Big 4" (LiDAR, Camera, IMU, F/T) and explicitly exclude GPS (indoor focus) and specialized sensors (thermal, audio) unless as brief mentions.
**Rationale**: These 4 form the minimum viable set for a general-purpose humanoid. GPS is irrelevant for indoor manipulation/balance.
**Alternatives Considered**: Including Tactile Skin. Rejected for detailed section because it's still largely research-grade, whereas F/T is standard industry hardware.

### 2. Physical Principles Depth
**Decision**: Focus on the "Intuitive Physics" (e.g., time-of-flight = echo, strain = bending) rather than equations.
**Rationale**: Target audience is CS/Engineering but the chapter is conceptual. Equations belong in specific module deep-dives (like the SLAM chapter later).

### 3. Sensor Fusion Approach
**Decision**: Use "Kalman Filter" concept as the mental model for fusion (predict-update cycle) without implementing the math.
**Rationale**: It provides the clearest intuition for *why* we fuse data (trusting sensors differently based on noise).

### 4. Hardware Examples
**Decision**: Cite specific, recognizable industry standards.
- LiDAR: Velodyne (historical), Ouster, Hesai.
- Camera: Intel RealSense, Oak-D (AI cameras).
- F/T: ATI Industrial Automation (gold standard), Robotiq (wrist).
- IMU: Xsens, MicroStrain.

## Implementation Strategy

- **Format**: MDX (Markdown).
- **Location**: `docs/part1/chapter4-sensor-systems.mdx`
- **Assets**: Diagrams for "Time of Flight" and "Sensor Fusion Pipeline".
