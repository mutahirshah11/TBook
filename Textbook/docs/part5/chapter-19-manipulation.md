---
sidebar_label: 'Chapter 19: Manipulation'
sidebar_position: 19
---

# Chapter 19: Manipulation and Grasping with Humanoid Hands

## Introduction

While walking gets a humanoid from A to B, manipulation is what makes it useful. Humanoid hands are "underactuated" or "high-DOF" systems designed for versatility. Unlike a simple parallel-jaw gripper, a 5-fingered hand can perform power grasps (hammer), precision pinches (needle), and in-hand manipulation (rotating a ball). This chapter covers the geometric foundations of grasping.

## Grasp Theory

To hold an object securely, we must satisfy force constraints.

### 1. Force Closure
A grasp is **Force Closure** if the fingers can apply forces to resist *any* external wrench (force + torque) applied to the object. Ideally, friction allows the fingers to push "tangentially".
-   **Friction Cone**: The set of all force vectors a finger can apply without slipping.
-   **Closure Condition**: The convex hull of all friction cones must contain the origin.

### 2. Form Closure
A stronger condition where the fingers physically cage the object (e.g., wrapping completely around a mug). The object cannot move even if friction is zero. Form closure is safer but harder to achieve with rigid objects.

### 3. The Grasp Matrix ($G$)
This matrix maps the contact forces $f$ to the net wrench $w$ on the object.
$$ w = G f $$
For a grasp to be stable, we check if the null space of $G$ allows for internal forces (squeezing) that keep the contacts active.

## Geometric Grasp Planning

Learning-based grasping (e.g., DexNet) is popular, but understanding **Analytical Grasping** is fundamental. We will build a planner that looks for **Antipodal Points**: two points on the object surface with surface normals pointing exactly opposite to each other.

### Heuristics for Good Grasps
1.  **Antipodal Alignment**: The line connecting the two contacts should be parallel to the surface normals.
2.  **CoM Proximity**: The grasp center should be close to the object's Center of Mass to minimize gravity torque.
3.  **Collision Free**: The approach path must not hit the table.

### Code Example: `grasp_planner.py`

```python
import numpy as np

def plan_antipodal_grasp(point_cloud, normals):
    """
    Simple analytical planner for a parallel-jaw grasp strategy.
    """
    best_grasp = None
    max_score = -1.0
    
    # Downsample for speed (randomly select 100 pairs)
    indices = np.random.choice(len(point_cloud), 100, replace=False)
    
    for i in indices:
        for j in indices:
            if i == j: continue
            
            p1 = point_cloud[i]
            p2 = point_cloud[j]
            n1 = normals[i]
            n2 = normals[j]
            
            # 1. Check Normal Alignment (should be opposite)
            # Dot product approx -1.0 means opposite
            alignment = np.dot(n1, -n2)
            if alignment < 0.9:
                continue
                
            # 2. Check Line Alignment
            # The line p1-p2 should be parallel to the normal n1
            grasp_axis = (p2 - p1) / np.linalg.norm(p2 - p1)
            axis_alignment = np.dot(n1, grasp_axis)
            if axis_alignment < 0.9:
                continue
                
            # 3. Score (Width + Alignment)
            width = np.linalg.norm(p2 - p1)
            if width > 0.08: # Gripper max width
                continue
                
            score = alignment + axis_alignment
            
            if score > max_score:
                max_score = score
                center = (p1 + p2) / 2
                # Construct rotation matrix from normal (Z-axis = approach)
                rotation = rotation_from_vector(n1)
                best_grasp = (center, rotation)
                
    return best_grasp
```

## Trajectory Planning with RRT*

Knowing *where* to grasp is half the battle. We also need to get there. **Rapidly-exploring Random Trees (RRT)** is the standard algorithm for high-DOF path planning.

1.  **Start**: Current joint configuration $q_{start}$.
2.  **Goal**: IK solution for grasp pose $q_{goal}$.
3.  **Loop**:
    -   Sample random $q_{rand}$.
    -   Find nearest node in tree $q_{near}$.
    -   Step towards $q_{rand}$ to get $q_{new}$.
    -   **Check Collision**: If $q_{new}$ is safe (self-collision or environment), add to tree.
    -   If close to $q_{goal}$, connect and smooth path.

## Dual-Arm Manipulation

Humanoids shine when tasks require two hands (e.g., unscrewing a jar, carrying a large box).
-   **Constraint Manifold**: The relative transform between the left and right hand is fixed by the object. This reduces the system's redundancy.
-   **Master-Slave Mode**: A simple control strategy where the Right Arm (Master) follows a trajectory, and the Left Arm (Slave) computes its IK to maintain the relative hold on the object.

## Summary

Manipulation is a search problem: searching for contact points on the surface (Grasp Planning) and searching for collision-free paths in joint space (Motion Planning). By combining geometric heuristics with RRT, we can robustly pick up novel objects.