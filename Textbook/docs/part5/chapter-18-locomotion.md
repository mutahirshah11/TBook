---
sidebar_label: 'Chapter 18: Locomotion'
sidebar_position: 18
---

# Chapter 18: Bipedal Locomotion and Balance Control

## Introduction

Bipedal locomotion is inherently unstable. A humanoid robot is essentially an inverted pendulum that is constantly trying to fall over. The goal of a walking controller is to place the feet in specific locations to "catch" the fall and redirect the momentum forward.

This chapter introduces the **Linear Inverted Pendulum Model (LIPM)** and **Model Predictive Control (MPC)**, the industry-standard approach for generating robust walking patterns.

## The Physics of Walking

### 1. The Linear Inverted Pendulum Model (LIPM)
If we constrain the robot's Center of Mass (CoM) to remain at a constant height $h$, the complex non-linear dynamics of the multi-body system collapse into a simple linear 2nd-order system.

The equation of motion is:
$$ \ddot{x} = \frac{g}{h} (x - p) $$
Where:
- $x$: CoM Position
- $\ddot{x}$: CoM Acceleration
- $p$: Zero Moment Point (ZMP)
- $g$: Gravity
- $h$: Constant height

This equation tells us that the CoM accelerates *away* from the ZMP. By controlling the ZMP (by shifting pressure on the foot), we control the CoM acceleration.

### 2. The Zero Moment Point (ZMP)
The ZMP is the point on the ground where the tipping moment due to gravity and inertia is zero.
- **Stability Criterion**: For the robot to remain flat-footed (not tip over), the ZMP must strictly lie within the **Support Polygon** (the convex hull of the feet on the ground).
- If the ZMP reaches the edge of the foot, the robot begins to rotate (fall).

## Walking Pattern Generation via MPC

Modern walking controllers don't just react to the current state; they look ahead. We use **Model Predictive Control (MPC)** to solve an optimization problem: *"What sequence of footsteps and ZMP locations will minimize jerk and keep the robot stable for the next 1.5 seconds?"*

### The Optimization Problem
We formulate this as a Quadratic Program (QP):

**Minimize**:
$$ J = \sum_{k=0}^{N} || p_k - p_{ref} ||^2 + \alpha || \dot{x}_k ||^2 + \beta || \dddot{x}_k ||^2 $$
*Minimize deviation from foot centers + Minimize velocity (stop at end) + Minimize jerk (smoothness).*

**Subject To**:
1.  **Dynamics**: $x_{k+1} = A x_k + B u_k$ (The LIPM physics).
2.  **ZMP Constraints**: $p_{min} \le p_k \le p_{max}$ (ZMP must be inside the foot).

### Code Example: 1D LIPM MPC with Cvxpy

This example generates a CoM trajectory to follow a sequence of footsteps.

```python
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

def solve_mpc_lipm(com_init, com_vel_init, footstep_centers):
    # 1. Constants
    h = 0.88        # CoM height (m)
    g = 9.81
    dt = 0.1        # Time step (s)
    N = 16          # Horizon (1.6 seconds preview)
    omega = np.sqrt(g/h)
    
    # 2. Discrete Dynamics (A, B matrices)
    # State = [x, v], Input = [zmp]
    # Discretization of x_ddot = w^2 (x - p)
    
    # 3. Optimization Variables
    # We optimize the ZMP locations for the next N steps
    zmp_ref = cp.Variable(N)
    com_pos = cp.Variable(N+1)
    com_vel = cp.Variable(N+1)
    
    constraints = []
    cost = 0
    
    # Initial State
    constraints += [com_pos[0] == com_init]
    constraints += [com_vel[0] == com_vel_init]
    
    for k in range(N):
        # Dynamics constraints (Euler Integration for simplicity)
        # acc = w^2 * (pos - zmp)
        acc = (omega**2) * (com_pos[k] - zmp_ref[k])
        
        constraints += [com_vel[k+1] == com_vel[k] + acc * dt]
        constraints += [com_pos[k+1] == com_pos[k] + com_vel[k] * dt]
        
        # Cost: Track the footstep centers
        # We assume footstep_centers is an array of length N matching the horizon
        cost += cp.square(zmp_ref[k] - footstep_centers[k])
        
    # Terminal Cost: Stop at the end
    cost += 10 * cp.square(com_vel[N])
    
    # 4. Solve
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve()
    
    return com_pos.value, zmp_ref.value

# --- Test the Solver ---
# Scenario: Robot starts at 0, needs to step to 0.3m, then 0.6m
footsteps = np.concatenate([np.zeros(5), np.ones(5)*0.3, np.ones(6)*0.6])
com_traj, zmp_traj = solve_mpc_lipm(0.0, 0.0, footsteps)

plt.plot(com_traj, label="CoM Trajectory")
plt.step(range(16), zmp_traj, label="ZMP Plan")
plt.legend()
plt.show()
```

## From Trajectory to Joints

The MPC gives us the desired Center of Mass trajectory. How do we move the motors?
1.  **Inverse Kinematics**: Use the IK solver from Chapter 17.
2.  **Tasks**:
    -   **Task 1 (High Priority)**: Keep feet on the ground (or moving to next step).
    -   **Task 2**: Move CoM to the MPC target $(x, y)$ and keep constant height $z$.
    -   **Task 3**: Keep trunk upright.

By solving this IK problem 100 times per second, the robot "walks" by shifting its hips to follow the computed CoM path while swinging its legs.

## Summary

We have reduced walking to a geometry problem: keep the ZMP inside the foot. By using MPC, we can anticipate future steps and start shifting weight early, resulting in the smooth, continuous motion characteristic of advanced humanoids.