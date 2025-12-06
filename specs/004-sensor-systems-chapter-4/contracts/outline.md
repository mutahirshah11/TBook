# Chapter 4 Outline Contract

**Target Audience**: CS/Engineering students; focus on "System Integration" view of sensors.
**Reading Time**: ~20 minutes.

## Section 1: Introduction
- **Hook**: The robot is blind, deaf, and numb without sensors.
- **Concept**: The "Perceptual Backbone" - sensors are the input layer of the AI.
- **Scope**: Focus on the four pillars: Vision, Range, Inertia, Force.

## Section 2: Exteroception (Seeing the World)
- **Subsection 2.1: LiDAR (The Long Range Scanner)**
  - Principle: Shooting lasers and counting time.
  - Use Case: Don't hit walls; map the room (SLAM).
  - Hardware: Spinning vs. Solid State.
- **Subsection 2.2: Cameras (The Semantic Eye)**
  - Principle: Capturing photons. RGB vs. RGB-D (Depth).
  - Use Case: "That is a cup" (Object Rec), "That is a face".
  - The Data: Dense pixel arrays vs. sparse LiDAR points.

## Section 3: Proprioception (Feeling the Self)
- **Subsection 3.1: IMU (The Inner Ear)**
  - Principle: MEMS accelerometers and gyros.
  - Criticality: You cannot balance a biped without an IMU.
  - The Challenge: Drift (integration error).
- **Subsection 3.2: Force/Torque (The Sense of Touch)**
  - Principle: Measuring strain. Wrist-mounted vs. Joint-torque sensors.
  - Use Case: Putting a peg in a hole; shaking hands safely.

## Section 4: Sensor Fusion (The Whole Picture)
- **Concept**: Why 1 + 1 = 3.
- **Example**: Visual-Inertial Odometry (VIO). Using camera to correct IMU drift; using IMU to handle fast motion blur.
- **The Pipeline**: Sensor -> Driver -> ROS 2 Topic -> Fusion Node -> State Estimate.

## Section 5: Summary & Bridge
- **Recap**: The sensor suite defines the robot's capabilities.
- **Bridge**: Now that we have the Body (Ch 3) and the Senses (Ch 4), we need a world to test them in. Next: Simulation.
