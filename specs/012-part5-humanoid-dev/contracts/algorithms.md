# Algorithm Interfaces

## 1. Kinematics Solver (IK)

**Function**: `solve_ik(model, data, target_frame_id, target_pose, initial_q)`

**Inputs**:
- `model`, `data`: Pinocchio objects.
- `target_frame_id`: Integer ID of the end-effector (e.g., hand).
- `target_pose`: `pin.SE3` object (Position + Rotation).
- `initial_q`: Numpy array (starting configuration).

**Outputs**:
- `q_sol`: Numpy array (solved joint configuration).
- `success`: Boolean.

**Contract**:
- The solver uses **CLIK (Closed-Loop Inverse Kinematics)** with Damped Least Squares (DLS) to handle singularities.
- Error tolerance: < 1mm position, < 1 deg rotation.

## 2. Walking Pattern Generator (MPC)

**Function**: `solve_mpc(current_com, current_zmp, footsteps)`

**Inputs**:
- `current_com`: `[x, y, dx, dy]` (Current state).
- `current_zmp`: `[px, py]` (Current ZMP).
- `footsteps`: List of future `(x, y)` locations for foot landings.

**Outputs**:
- `com_traj`: List of future CoM states.
- `zmp_traj`: List of future ZMP references.

**Contract**:
- Uses **LIPM** dynamics (constant height).
- Optimization horizon: 1.0 - 1.5 seconds.
- Solver: `cvxpy` or `quadprog`.

## 3. HRI Command Parser

**Function**: `parse_audio(audio_stream)`

**Inputs**:
- `audio_stream`: Raw bytes (16kHz, Mono).

**Outputs**:
- `intent`: Enum (`WAVE`, `GRASP`, `WALK`).
- `params`: Dict (e.g., `{ "target": "red ball" }`).

**Contract**:
- Keyword spotting latency < 500ms.
- Must robustly reject background noise (handled by Vosk model).
