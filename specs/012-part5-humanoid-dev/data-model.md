# Data Model: Part 5 - Humanoid Robot Development

**Feature**: `012-part5-humanoid-dev` | **Date**: 2025-12-07

## Entities

### Humanoid Model (Pinocchio)
*Represents the physical robot state.*

| Entity | Description | Attributes |
| :--- | :--- | :--- |
| **Model** | The static kinematic tree (joints, bodies). | `nq` (config size), `nv` (velocity size)<br>`joints`: list<br>`frames`: list |
| **Data** | The dynamic workspace (updated every step). | `oMi` (joint placements)<br>`oMf` (frame placements)<br>`M` (mass matrix) |
| **Configuration (`q`)** | The joint angles vector. | `[x, y, z, qx, qy, qz, qw, joint_1 ... joint_n]` (Size: 39 for Talos) |
| **Velocity (`v`)** | The joint velocities vector. | `[vx, vy, vz, wx, wy, wz, v_1 ... v_n]` (Size: 38 for Talos) |

### Locomotion State
*Represents the simplified model for walking control.*

| Entity | Description | Attributes |
| :--- | :--- | :--- |
| **CoM State** | Center of Mass position and velocity. | `[c_x, c_y, c_z, dc_x, dc_y, dc_z]` |
| **ZMP** | Zero Moment Point on the ground. | `[p_x, p_y]` |
| **Footstep** | A target placement for a foot. | `position`: (x,y,z)<br>`rotation`: (yaw)<br>`time`: float |

### Interaction
*Represents the HRI state.*

| Entity | Description | Attributes |
| :--- | :--- | :--- |
| **VoiceCommand** | A parsed instruction from the user. | `intent`: string (e.g., "GRASP")<br>`target`: string (e.g., "CUP") |
| **GraspCandidate** | A planned hand pose. | `pose`: SE3 (Position + Orientation)<br>`quality`: float (0-1) |

## Validation Rules

1. **Free-Flyer**: The first joint of the humanoid `q` vector MUST be a 7D vector (3 pos + 4 quat) representing the floating base.
2. **ZMP Constraint**: The computed ZMP must always lie within the Convex Hull of the support polygon (the active feet on the ground).
3. **Joint Limits**: Calculated IK solutions must respect `model.lowerPositionLimit` and `model.upperPositionLimit`.

## State Transitions

### Walking Cycle
1.  **Double Support**: Both feet on ground. ZMP moves from trailing foot to leading foot.
2.  **Single Support**: One foot lifts (Swing Phase). ZMP must stay inside the Stance Foot polygon.
3.  **Impact**: Swing foot lands. Switch back to Double Support.
