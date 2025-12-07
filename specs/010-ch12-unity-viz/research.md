# Research: Chapter 12 - Unity for Robot Visualization

**Feature**: `010-ch12-unity-viz` | **Date**: 2025-12-07

## Decisions

### 1. Unity Installation Method
- **Decision**: Use **Unity Hub** to install **Unity 2022.3 LTS**. Install `com.unity.robotics.ros-tcp-connector` via **Git URL** in Package Manager.
- **Rationale**: 
  - Unity Hub is the standard, user-friendly entry point.
  - 2022.3 LTS provides long-term stability and compatibility with current Robotics Hub packages.
  - Git URL is the direct and officially documented method for the Robotics Hub packages, avoiding the need for manual file copying.
- **Alternatives Considered**: 
  - *Download & Copy*: Error-prone for beginners.
  - *Unity Asset Store*: Packages often lag behind the GitHub repo.

### 2. Networking Configuration
- **Decision**: **Host-to-VM/WSL IP** Strategy.
  - **Unity (Windows Host)**: Connects to WSL2 IP on port 10000.
  - **ROS2 (WSL2)**: `ros_tcp_endpoint` listens on `0.0.0.0` to accept external connections.
  - **Firewall**: Explicit instruction to add an Inbound Rule for TCP port 10000 on Windows.
- **Rationale**: 
  - "Mirrored Mode" is excellent but requires Windows 11 22H2+, leaving many Windows 10 users behind. A manual IP approach covers *all* users.
  - `localhost` often fails in standard WSL2 networking, leading to user frustration.
- **Alternatives Considered**: 
  - *Mirrored Mode*: Simpler but limits audience.
  - *Docker Bridge*: Too complex for an introductory chapter.

### 3. URDF Import Pipeline
- **Decision**: Standard `urdf-importer` with a mandatory **"Render Pipeline Converter"** step.
- **Rationale**: 
  - Importing standard URDFs into URP results in pink (broken) materials because they use Built-in shaders.
  - The "Render Pipeline Converter" is a built-in Unity tool that reliably fixes this with one click.
- **Alternatives Considered**: 
  - *Manual Shader Swap*: Too tedious for complex robots.

### 4. Physics Engine
- **Decision**: Exclusively use **ArticulationBody**.
- **Rationale**: 
  - Designed specifically for robotics (stable kinematic chains, 1000:1 mass ratios).
  - Reduced jitter compared to Rigidbody/ConfigurableJoint.
  - "Digital Twin" accuracy requires the precise joint drive models inherent to ArticulationBody.
- **Alternatives Considered**: 
  - *Rigidbody*: Prone to "exploding" simulations with complex joint chains.

## Unknowns Resolved

- **Unity Version**: Confirmed 2022.3 LTS is supported.
- **Installation**: Git URL is the correct path.
- **Networking**: Specific firewall command identified (`New-NetFirewallRule`).
- **URP Compatibility**: Material upgrader tool is the solution.
- **Physics**: ArticulationBody is the clear winner for stability.
