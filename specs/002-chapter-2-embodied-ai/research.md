# Research: Chapter 2 — Embodied AI and Physical Laws

**Status**: Complete
**Date**: 2025-12-05

## Decisions

### 1. Mathematical Formalism
*   **Decision**: Use Discrete Time State-Space notation.
*   **Notation**:
    *   State: $x_t$ (Vector describing the robot configuration, e.g., position, velocity)
    *   Control Input: $u_t$ (Vector of motor commands/torques)
    *   Dynamics Function: $x_{t+1} = f(x_t, u_t)$ (The "Laws of Physics" function)
    *   Observation: $y_t = h(x_t)$ (Sensors)
*   **Rationale**: This is the standard language of modern robotics (Control Theory and RL). It allows defining "Dynamics" ($f$) without needing to write out complex differential equations for specific robots immediately. It aligns with the "Light Formalism" requirement.

### 2. Examples for "Digital vs. Physical"
*   **Example 1**: *The Undo Button*.
    *   Digital: Typing a document -> Ctrl+Z is perfect.
    *   Physical: Pouring milk -> "Undo" is impossible (Irreversibility).
*   **Example 2**: *The Frozen Screen*.
    *   Digital: Video game lags -> Character teleports or pauses.
    *   Physical: Robot controller lags -> Robot crashes into wall due to momentum ($p=mv$). Physics doesn't pause.
*   **Example 3**: *Data vs. Energy*.
    *   Digital: Copying a file costs negligible energy.
    *   Physical: Moving an arm costs Joules/Watts. Battery life dictates mission success.

### 3. Transition Strategy (Interwoven Recap)
*   **Decision**: Use "Callout Blocks" (Admonitions in Docusaurus) titled "Recall from Chapter 1".
*   **Key Linkages**:
    *   When defining "Embodiment", recall Ch1's definition of "Physical AI".
    *   When discussing "Sensing/Perception", recall Ch1's "Sense-Think-Act" loop.

### 4. Uncertainty & Noise Handling
*   **Decision**: Introduce the concept of "The Fog of War" in robotics.
*   **Concepts**:
    *   Sensor Noise (Measurement error).
    *   Actuation Noise (Motors not doing exactly what told).
    *   Process Noise (Wind, uneven terrain).
*   **Rationale**: Prepares reader for Probabilistic Robotics (State Estimation) in later chapters.

## Alternatives Considered
*   *Continuous Time ($\dot{x} = f(x,u)$)*: Rejected. Requires calculus knowledge (derivatives) which might alienate some readers. Discrete time is more intuitive for computer scientists (loops).
*   *Specific Robot Dynamics (e.g., Inverted Pendulum equations)*: Rejected. Too much math too early. Keep it generic $f(x,u)$.
