---
sidebar_label: 'Chapter 17: Kinematics & Dynamics'
sidebar_position: 17
---

# Chapter 17: Humanoid Robot Kinematics and Dynamics

## Introduction

Humanoid robots are among the most complex mechanical systems to model. Unlike industrial arms which are fixed to a sturdy base, humanoids have a **floating base**—the pelvis moves freely in 3D space, constrained only by contacts with the environment. This chapter dives deep into the mathematical foundations required to control these systems, focusing on the modern, high-performance **Pinocchio** library.

## The Pinocchio Library

**Pinocchio** is a rigid body dynamics library designed for efficiency and derivative computation. It is the standard tool in the humanoid robotics research community (used by LAAS-CNRS, NYU, and PAL Robotics) because it separates the *model* (constants) from the *data* (buffers), allowing for zero-allocation runtime loops.

### Setup Guide

To run the examples in this chapter, we will use a dedicated Conda environment. We also install `example-robot-data` to get the URDF for the TALOS robot without needing to clone complex Git repositories.

```bash
conda create -n humanoid python=3.10
conda activate humanoid
# Install Pinocchio and the Talos robot assets
conda install -c conda-forge pinocchio meshcat example-robot-data
```

## Mathematical Foundations

### 1. The Floating Base and SE(3)
The state of a humanoid is defined by the configuration of its joints plus the position and orientation of its root. We describe this root state using the **Special Euclidean Group SE(3)**.

-   **Position**: A 3D vector $p \in \mathbb{R}^3$.
-   **Orientation**: A rotation matrix $R \in SO(3)$ or a unit quaternion.

In Pinocchio, the configuration vector $q$ has a specific structure:
$$ q = [\underbrace{x, y, z, q_x, q_y, q_z, q_w}_{\text{Free Flyer (7)}}, \underbrace{q_1, \dots, q_n}_{\text{Joints}}] $$

The first 7 elements represent the root. Note that Pinocchio uses `(x, y, z, w)` order for quaternions in some contexts but stores them as `(x, y, z)` vector part + `w` scalar part in the configuration vector.

### 2. Pinocchio Architecture: Model vs. Data
-   **Model**: Contains constant parameters (link lengths, masses, inertias). It is read-only during simulation.
-   **Data**: Contains working memory (velocity vectors, acceleration, temporary matrices). You create a `Data` object from a `Model`.

This design allows you to run multiple parallel simulations (e.g., for MPC or Reinforcement Learning) using a single `Model` and multiple `Data` instances.

## Forward Kinematics (FK)

FK answers the question: *"Given the joint angles, where is the hand?"*

In a floating base system, FK is computed relative to the world frame, compounding the root transformation with the kinematic chain.

```python
import pinocchio as pin
import example_robot_data as erd
import numpy as np

# 1. Load the Robot Model
robot = erd.load("talos")
model = robot.model
data = model.createData()

# 2. Sample a Random Configuration
q = pin.randomConfiguration(model)

# 3. Compute FK
# This updates the 'oMi' (placement relative to world) for all joints
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

# 4. Query a specific Frame
hand_id = model.getFrameId("arm_right_7_link")
hand_placement = data.oMf[hand_id]

print(f"Hand Translation: {hand_placement.translation.T}")
print(f"Hand Rotation:\n{hand_placement.rotation}")
```

## Inverse Kinematics (IK)

IK answers: *"What joint angles place the hand at target $X$?"*

For humanoids, analytical IK (geometric formulas) is impossible due to the high number of degrees of freedom (DoF). Instead, we use **Numerical IK**, specifically **Closed-Loop Inverse Kinematics (CLIK)**.

### The Jacobian
The Jacobian $J(q)$ relates joint velocities $\dot{q}$ to end-effector spatial velocity $v$.
$$ v = J(q) \dot{q} $$

We solve for $\dot{q}$ using the Moore-Penrose pseudoinverse $J^\dagger$, often regularized with a damping factor to handle singularities (positions where the robot loses a degree of freedom, like a fully outstretched arm).

### Code Example: Damped Least Squares Solver

```python
def solve_ik(model, data, target_frame, target_pose, q_init):
    q = q_init.copy()
    dt = 1e-1       # Virtual time step
    damping = 1e-12 # Regularization
    
    for i in range(1000):
        # Update Kinematics
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        
        # 1. Compute Error (in SE3, this is a 6D twist)
        current_pose = data.oMf[target_frame]
        # Log maps the difference between two poses to a 6D vector
        error = pin.log(current_pose.inverse() * target_pose).vector
        
        if np.linalg.norm(error) < 1e-4:
            print(f"Converged in {i} iterations")
            return q, True
            
        # 2. Compute Jacobian (in the local frame of the end-effector)
        J = pin.computeFrameJacobian(model, data, q, target_frame)
        
        # 3. Solve: J * dq = -error
        # Using Damped Least Squares: dq = J^T * (J * J^T + lambda * I)^-1 * (-error)
        v = -J.T.dot(np.linalg.solve(J.dot(J.T) + damping * np.eye(6), error))
        
        # 4. Integrate Configuration
        # pin.integrate handles the quaternion math correctly for the floating base
        q = pin.integrate(model, q, v * dt)
        
    return q, False
```

## Dynamics Algorithms

Understanding forces is crucial for balance. Pinocchio implements the "Rigid Body Dynamics Algorithms" (Featherstone).

### 1. Recursive Newton-Euler Algorithm (RNEA)
**Inverse Dynamics**: $ \tau = \text{RNEA}(q, v, a) $
Given the motion, what torques are required? This is used in the **Whole-Body Controller** to compute the feed-forward torques needed to execute a trajectory.

### 2. Composite Rigid Body Algorithm (CRBA)
**Mass Matrix**: $ M(q) = \text{CRBA}(q) $
Computes the joint-space inertia matrix. Essential for computing the kinetic energy of the system.

### 3. Articulated Body Algorithm (ABA)
**Forward Dynamics**: $ a = \text{ABA}(q, v, \tau) $
Given the torques, how will the robot move? This is the core of any physics simulator. Pinocchio's ABA is heavily optimized and often faster than generic physics engines.

## Summary

By mastering SE(3) math and the Jacobian pseudo-inverse, you now have the tools to position a humanoid's limbs arbitrarily. In the next chapter, we will apply these concepts to the hardest problem in humanoid robotics: keeping the robot upright while walking.
