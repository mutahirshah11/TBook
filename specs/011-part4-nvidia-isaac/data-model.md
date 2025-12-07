# Data Model: Part 4 - NVIDIA Isaac Platform

**Feature**: `011-part4-nvidia-isaac` | **Date**: 2025-12-07

## Entities

### Isaac Sim Scene (USD)
*Represents the simulation world hierarchy.*

| Entity | Description | Attributes |
| :--- | :--- | :--- |
| **Stage** | The root container for the simulation. | `up_axis`: Z-up (Standard in Isaac)<br>`units`: Meters |
| **Prim (Primitive)** | Basic building block (mesh, light, xform). | `path`: string (e.g., `/World/Robot`)<br>`type`: string (e.g., `UsdGeom.Xform`) |
| **Rigid Body** | Physics property applied to Prims. | `mass`: float<br>`collider`: Mesh/Cube/Sphere approximation |
| **Articulation** | Robot kinematic chain (URDF import result). | `joints`: list[Joint]<br>`dof`: int (Degrees of Freedom) |

### RL Environment (Isaac Lab)
*Represents the training configuration and state.*

| Entity | Description | Attributes |
| :--- | :--- | :--- |
| **ManagerBasedRLEnv** | The gym-compatible environment class. | `cfg`: EnvCfg object |
| **EnvCfg** | Configuration dataclass. | `decimation`: int (sim steps per policy step)<br>`episode_length_s`: float |
| **Observation** | Input to the policy network. | `joint_pos`: tensor<br>`joint_vel`: tensor<br>`goal_pos`: tensor |
| **Action** | Output from the policy. | `joint_effort`: tensor (Torque/Position target) |
| **Reward** | Scalar signal for learning. | `track_lin_vel_xy_exp`: function (Tracking error)<br>`lin_vel_z_l2`: function (Penalize vertical movement) |

### Synthetic Data (Replicator)
*Represents the generated dataset structure.*

| Entity | Description | Attributes |
| :--- | :--- | :--- |
| **RenderProduct** | A camera Viewport + Resolution. | `resolution`: (W, H) |
| **Annotator** | The "sensor" capturing ground truth. | `type`: "rgb", "semantic_segmentation", "depth" |
| **Semantics** | Labels attached to 3D objects. | `class`: "cube", "robot", "background" |

## Validation Rules

1. **Semantics**: For the Semantic Segmentation annotator to produce non-black output, Prims MUST have semantic labels applied via `rep.get.semantics.add_semantics()`.
2. **Physics**: RL training requires the `Articulation` root to have a fixed base (unless it's a mobile robot) and proper collision meshes.
3. **Device**: All tensors in Isaac Lab MUST be on the same CUDA device (e.g., `cuda:0`). CPU tensors will cause runtime errors in the PhysX pipeline.

## State Transitions

### RL Training Loop
1.  **Reset**: Simulation resets robots to initial state + applies **Startup Domain Randomization**.
2.  **Step**:
    - Policy computes `actions` from `observations`.
    - Simulation steps physics `decimation` times.
    - **Interval Domain Randomization** may trigger (e.g., gravity change).
    - New `observations` and `rewards` are computed.
3.  **Termination**: If `episode_length` exceeded or termination condition met (e.g., fall), trigger Reset.
