# Research: Part 5 - Humanoid Robot Development

**Feature**: `012-part5-humanoid-dev` | **Date**: 2025-12-07

## Decisions

### 1. Kinematics Library
- **Decision**: **Pinocchio** installed via `conda install pinocchio -c conda-forge`.
- **Rationale**:
  - It is the modern standard for efficient rigid body dynamics in robotics (used by LAAS-CNRS, NYU, etc.).
  - Native Python bindings are excellent and performant (C++ backend).
  - Supports "free-flyer" joints out of the box, which is critical for humanoid floating-base dynamics.
- **Alternatives Considered**:
  - *KDL*: Slower, less pythonic, harder to handle complex dynamics.
  - *PyBullet*: Good for sim, but less specialized for pure kinematic/dynamic solver implementation.

### 2. Robot Model
- **Decision**: **PAL Robotics Talos** via `example-robot-data`.
- **Rationale**:
  - `example-robot-data` is a standard package maintained by the Pinocchio developers that includes ready-to-use URDFs for Talos, Tiago, Solo, etc.
  - Talos is a full-sized, torque-controlled humanoid, making it perfect for discussing both locomotion and manipulation.
- **Access**: `import example_robot_data as erd; robot = erd.load('talos')`.

### 3. Locomotion Control Strategy
- **Decision**: **LIPM (Linear Inverted Pendulum Mode) + MPC (Model Predictive Control)**.
- **Rationale**:
  - LIPM is the standard simplified model for teaching walking.
  - MPC provides a robust way to handle constraints (ZMP bounds) and lookahead, which is more modern and effective than static analytical solutions.
  - A Python implementation using `cvxpy` or `quadprog` is educational and readable.
- **Alternatives Considered**:
  - *Static Walking*: Too slow/limited for a "modern" robotics book.
  - *Whole-Body Control (WBC)*: Too complex for a single chapter; better to focus on the centroidal dynamics (MPC) first.

### 4. HRI Keyword Spotting
- **Decision**: **Vosk** (`pip install vosk`).
- **Rationale**:
  - Truly offline, no API keys required.
  - Lightweight enough for a textbook example but accurate enough for a demo.
  - Simple Python API: `rec.AcceptWaveform(data)`.
  - Pre-trained models are small (~50MB) and easy to download.
- **Alternatives Considered**:
  - *OpenWakeWord*: Good, but Vosk offers full speech-to-text which enables command parsing ("Pick up the *red* cup"), not just wake words.
  - *Pocketsphinx*: Outdated accuracy.

## Unknowns Resolved

- **Installation**: Conda is the preferred route for Pinocchio to handle system dependencies.
- **Assets**: `example-robot-data` eliminates the need to manually clone/fix Git repositories for URDFs.
- **MPC Solver**: `cvxpy` with `OSQP` backend is the most readable/robust choice for Python MPC.
