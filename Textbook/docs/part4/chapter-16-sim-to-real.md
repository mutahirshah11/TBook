---
sidebar_label: 'Chapter 16: Sim-to-Real'
sidebar_position: 16
---

# Chapter 16: Sim-to-Real Transfer Techniques

## The Reality Gap

The "Reality Gap" is the infamous drop in performance when an AI model trained in simulation fails in the real world. It stems from two main sources:

1.  **Dynamics Gap**: The physics simulator is an approximation. Real motors have friction, backlash, and heat-based dampening that `F=ma` doesn't perfectly capture. Real floors are uneven; real contacts are soft.
2.  **Visual Gap**: If using cameras, real light is messy. Shadows, reflections, and "grain" in the image sensor look different from the perfect, crisp renders of Isaac Sim.

## Domain Randomization (DR)

We cannot model reality perfectly. Instead, we embrace chaos. **Domain Randomization** is the technique of varying simulation parameters during training across a wide range. Ideally, the "Real World" becomes just one sample within that distribution.

### Types of Randomization

- **Physics Randomization**:
    - **Mass**: Vary robot link mass by ±10%.
    - **Friction**: Change floor friction from "Ice" (0.1) to "Carpet" (1.0).
    - **Damping**: Add resistance to joints to simulate rusty motors.
- **Visual Randomization** (for camera-based policies):
    - **Lighting**: Randomize position, color, and intensity of lights.
    - **Texture**: Swap the floor texture with random images.
    - **Camera Pose**: Jitter the camera position slightly to simulate mounting errors.

### Code Example: `domain_randomization_cfg.py`

In Isaac Lab, we use the `EventTermCfg` to inject these randomizations into the environment loop.

```python
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
import isaaclab.envs.mdp as mdp

@configclass
class EventCfg:
    # 1. Mass Randomization
    # This forces the policy to be robust to weight estimation errors
    add_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup", # Apply once when env is created
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "mass_distribution_params": (-0.5, 0.5), # Add between -0.5kg and +0.5kg
            "operation": "add"
        }
    )

    # 2. Friction Randomization
    # Crucial for locomotion - robot must learn not to slip
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.4, 1.0),
            "dynamic_friction_range": (0.4, 0.9),
            "restitution_range": (0.0, 0.1), # Bounciness
        }
    )
    
    # 3. Push Randomization
    # Periodically shove the robot to teach it balance recovery
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval", # Apply every N seconds
        interval_range_s=(10.0, 15.0),
        params={
            "velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)},
        }
    )
```

## System Identification (System ID)

Before randomizing, we need a good "center point." **System ID** is the process of measuring your real robot to tune the simulation base parameters.

- **Weigh individual parts**: Don't trust the CAD file blindly.
- **Motor Identification**: Record torque-vs-velocity curves on the real motor and fit a curve to match in sim.
- **Latency Measurement**: Measure the time from "Python Command" to "Motor Movement". Add this delay (often 20-50ms) to the simulation step logic.

### Advanced: Actuator Nets
For high-performance robots, a simple DC Motor model isn't enough. Researchers often train a small neural network (an **Actuator Net**) to predict the motor's response to a command, training it on real-world log data. This net is then used inside the simulator as the physics model for the joint.

## Deployment

Once trained, you export your policy (usually a small neural network) to a format like **ONNX** or **TorchScript**. This file is loaded onto the real robot's onboard computer (e.g., Jetson Orin).

The real robot code runs a simple loop:
1.  **Read Sensors**: Get current Joint Positions (q) and Velocities (dq).
2.  **Normalize Data**: Apply the exact same scaling/normalization you used in `ObservationManager`.
3.  **Run Inference**: Pass the normalized vector to the ONNX model.
4.  **Scale Output**: Convert the raw neural net output [-1, 1] to Torque commands [Nm].
5.  **Safety Filter**: Clip the commands to safe limits (e.g., max torque, max velocity) to prevent hardware damage if the policy hallucinates.
6.  **Send to Motors**: Dispatch the command over CAN bus or EtherCAT.

If your Domain Randomization was sufficient, the robot should behave similarly to how it did in Isaac Sim!
