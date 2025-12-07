# Quickstart: Part 5 - Humanoid Robot Development

**Prerequisites**:
- Python 3.8 or higher
- Conda (Anaconda or Miniconda)

## 1. Environment Setup

Create a fresh environment to avoid conflicts with system packages.

```bash
conda create -n humanoid python=3.10
conda activate humanoid
```

## 2. Install Core Libraries

We use `conda-forge` for the robotics stack.

```bash
# Install Pinocchio, Meshcat (visualizer), and example robots
conda install -c conda-forge pinocchio meshcat example-robot-data

# Install optimization and math tools
conda install numpy scipy matplotlib

# Install QP solver for MPC
pip install cvxpy
```

## 3. Install HRI Tools

```bash
# Keyword spotting
pip install vosk sounddevice

# Computer Vision (MediaPipe for pose)
pip install mediapipe opencv-python
```

## 4. Verify Installation

Run this python snippet to ensure Pinocchio can load the Talos robot.

```python
import pinocchio as pin
import example_robot_data as erd
import sys

try:
    # Load Robot
    robot = erd.load("talos")
    print(f"Successfully loaded {robot.model.name}")
    print(f" - DoF: {robot.model.nv}")
    print(f" - Joints: {robot.model.njoints}")
    
    # Visualize (Optional)
    # robot.initViewer(loadModel=True)
    # robot.display(robot.q0)
    
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
```

If you see "Successfully loaded talos", you are ready for Chapter 17.
