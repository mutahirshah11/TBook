# Isaac Lab Interaction Contract

**Framework**: Isaac Lab (PyTorch + Omniverse)
**Interface**: OpenAI Gym / Gymnasium Standard (`reset()`, `step()`)

## Environment Interface

### Input: Actions
**Shape**: `(num_envs, num_dof)`
**Type**: `torch.Tensor` (float32)
**Device**: `cuda:0`
**Semantics**:
- Continuous values representing joint targets.
- Depending on `ActionTermCfg`, this could be:
  - Joint Positions (in radians)
  - Joint Velocities (in rad/s)
  - Joint Torques (in Nm)

### Output: Observations
**Shape**: `(num_envs, obs_dim)` (e.g., 48 for Ant)
**Type**: `torch.Tensor` (float32)
**Device**: `cuda:0`
**Components**:
- Base linear velocity (3)
- Base angular velocity (3)
- Projected gravity vector (3)
- Joint positions (num_dof)
- Joint velocities (num_dof)
- Previous actions (num_dof)

### Output: Rewards
**Shape**: `(num_envs,)`
**Type**: `torch.Tensor` (float32)
**Calculation**: Sum of weighted reward terms defined in `RewardsCfg`.

## Replicator API Contract

### Setup
**Call**: `rep.create.render_product(camera, (W, H))`
**Return**: `RenderProduct` path string.

### Annotator Access
**Call**: `annotator.get_data()`
**Return**: Dictionary (numpy arrays)
- **RGB**: `(H, W, 4)` uint8 array (RGBA).
- **Semantic Segmentation**: `(H, W)` uint32 array (Instance/Class IDs).
- **Depth**: `(H, W)` float32 array (Distance in meters).

## Sim-to-Real Config Contract

**Config Class**: `EventTermCfg`
**Fields**:
- `func`: Python callable (e.g., `randomize_mass`).
- `mode`: "startup" | "reset" | "interval".
- `params`: Dictionary of arguments.

**Example Contract**:
```python
EventTermCfg(
    func=randomize_rigid_body_mass,
    mode="reset",
    params={
        "asset_cfg": SceneEntityCfg("robot"),
        "mass_range": (0.8, 1.2), # Contract: Multiplier relative to default mass
        "operation": "scale"
    }
)
```
