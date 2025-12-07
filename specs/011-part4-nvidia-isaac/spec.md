## Clarifications

### Session 2025-12-07
- Q: RL Framework Selection? → A: **Isaac Lab** (Official successor, built on Isaac Sim 4.0+, future-proof).
- Q: Isaac Sim Version Target? → A: **Isaac Sim 4.0.0** (Specific version for stability and reproducibility).
- Q: OS Compatibility Scope? → A: **Linux (Ubuntu) Focus** (Simplifies installation/RL instructions, industry standard).
- Q: Manipulation Robot Model Choice? → A: **Franka Emika Panda** (Common, well-supported in Isaac Sim).
- Q: Replicator API Usage? → A: **Python Scripting (Programmatic)** (Preferred for automation and reproducibility).

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Chapter 13: Introduction to NVIDIA Isaac (Priority: P1)

As a reader, I want to understand the ecosystem of NVIDIA Isaac SDK and Isaac Sim so that I can set up a simulation environment for advanced robotics.

**Why this priority**: Foundational knowledge required for subsequent chapters in this part.

**Independent Test**: Reader can install Isaac Sim and run a basic "Hello World" simulation example.

**Acceptance Scenarios**:

1. **Given** a workstation with an NVIDIA RTX GPU, **When** the reader follows the setup guide, **Then** NVIDIA Omniverse Launcher and Isaac Sim are successfully installed.
2. **Given** a running Isaac Sim instance, **When** the reader loads the "Hello World" sample, **Then** a robot is visible and can be controlled via basic keyboard teleoperation or a python script.

---

### User Story 2 - Chapter 14: AI-Powered Perception and Manipulation (Priority: P2)

As a reader, I want to use Isaac Sim's perception tools to train or validate a manipulation task so that I can perform tasks like pick-and-place with computer vision.

**Why this priority**: Demonstrates the core value proposition of Isaac (AI + Simulation).

**Independent Test**: Reader can configure a camera sensor in Isaac Sim and retrieve labeled synthetic data (RGB + Segmentation).

**Acceptance Scenarios**:

1. **Given** a scene with a manipulator robot and objects, **When** the reader adds a Camera sensor, **Then** the sensor publishes RGB images to a ROS2 topic (or Python API).
2. **Given** the camera stream, **When** the reader enables Semantic Segmentation, **Then** the output images contain color-coded masks for the objects.

---

### User Story 3 - Chapter 15: Reinforcement Learning for Control (Priority: P2)

As a reader, I want to train a robot policy using Reinforcement Learning (**Isaac Lab**) so that the robot learns a behavior (e.g., balancing or walking) without manual coding.

**Why this priority**: Covers the "Sim-to-Real" training pipeline which is a major trend in modern robotics.

**Independent Test**: Reader can launch a training script (e.g., Cartpole or Ant) and observe the reward metric increasing over time.

**Acceptance Scenarios**:

1. **Given** the **Isaac Lab** environment, **When** the reader starts the training script, **Then** the simulation runs in headless/visual mode and logs training progress (Tensorboard/WandB).
2. **Given** a trained policy checkpoint, **When** the reader runs the inference script, **Then** the robot successfully performs the task (e.g., balances the pole).

---

### User Story 4 - Chapter 16: Sim-to-Real Transfer Techniques (Priority: P3)

As a reader, I want to understand how to transfer simulation-trained policies to real hardware so that I can bridge the "reality gap".

**Why this priority**: Closes the loop on the simulation learning process.

**Independent Test**: Reader can apply domain randomization parameters to a simulation scene and observe the visual/physical variance.

**Acceptance Scenarios**:

1. **Given** a simulation scene, **When** the reader applies Domain Randomization (DR), **Then** properties like lighting, textures, and friction coefficients change randomly between episodes.
2. **Given** a trained model, **When** the reader deploys it (conceptually or to a mock hardware interface), **Then** the system handles noisy sensor data robustly.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter 13 MUST provide installation instructions for NVIDIA Omniverse and Isaac Sim (targeting **v4.0.0**) with a primary focus on **Linux (Ubuntu)**.
- **FR-002**: Chapter 13 MUST explain the relationship between Isaac SDK (legacy/maintenance) and the new Isaac Sim based on Omniverse.
- **FR-003**: Chapter 14 MUST demonstrate setting up a manipulation scene with a **Franka Emika Panda** robot and rigid body physics.
- **FR-004**: Chapter 14 MUST guide the reader through generating synthetic data (RGB-D, Segmentation) using the **Replicator Python API**.
- **FR-005**: Chapter 15 MUST explain the basics of RL in robotics (Agent, Environment, Reward, State).
- **FR-006**: Chapter 15 MUST provide a tutorial on training a policy using NVIDIA's RL framework (**Isaac Lab**).
- **FR-007**: Chapter 16 MUST explain the "Reality Gap" and common techniques to mitigate it (Domain Randomization, System Identification).
- **FR-008**: Chapter 16 MUST demonstrate how to configure Domain Randomization in Isaac Sim.

### Key Entities

- **Omniverse Nucleus**: Local server for asset management.
- **USD (Universal Scene Description)**: The file format used by Isaac Sim.
- **Isaac Lab**: The RL training framework built on Isaac Sim.
- **Replicator**: The synthetic data generation tool.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A user with a compliant GPU (RTX 2070+) can install Isaac Sim and reach a running state in under 1 hour.
- **SC-002**: The **Isaac Lab** RL training example (Ch 15) converges to a successful policy within 10,000 iterations (or <20 mins on standard hardware).
- **SC-003**: The synthetic data generation (Ch 14) produces at least 100 labeled frames in a single batch run.
- **SC-004**: Each chapter includes at least one "Hands-On" exercise block.

## Assumptions

- The reader has access to a computer with an NVIDIA RTX GPU (Required for Isaac Sim).
- The reader is familiar with basic Python programming.
- The reader understands basic robotics concepts (frames, joints) from Part 1/2.
- We will focus on Isaac Sim (Omniverse) rather than the older Isaac SDK, as SDK is being deprecated in favor of ROS2 + Isaac Sim.
- The primary target operating system for all instructions will be **Linux (Ubuntu)**, with notes on potential Windows differences.