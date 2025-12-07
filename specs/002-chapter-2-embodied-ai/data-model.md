# Data Model: Chapter 2 Concepts

**Status**: Draft
**Date**: 2025-12-05

## Core Entities (Concepts)

### 1. The Embodied Agent
*   **Definition**: An intelligent system constrained by a physical body and environment.
*   **Attributes**:
    *   **Morphology**: The physical shape and structure (Hardware).
    *   **State ($x$)**: The internal condition (Position, Velocity).
    *   **Sensors**: Interface to perceive the world ($y$).
    *   **Actuators**: Interface to change the world ($u$).

### 2. The Environment
*   **Definition**: The physical world the agent interacts with.
*   **Attributes**:
    *   **Laws of Physics**: Immutable rules (Gravity, Friction).
    *   **Stochasticity**: Unpredictable elements (Noise, Disturbances).
    *   **Energy Source**: Where the agent gets power (Battery, Cable).

### 3. The Dynamics Function ($f$)
*   **Definition**: The mapping that determines how the state evolves over time.
*   **Equation**: $x_{t+1} = f(x_t, u_t) + \epsilon$
    *   $x_t$: Current state
    *   $u_t$: Action taken
    *   $\epsilon$: Noise/Uncertainty

### 4. Physical Constraints (The "Big Three")
1.  **Dynamics**: You cannot move instantly. $F=ma$ means acceleration takes time and force.
2.  **Energy**: Every action costs power. Power is finite.
3.  **Uncertainty**: You never know exact state ($x$) or exact outcome ($f$).

## Relationships
*   **Agent** *interacts with* **Environment** via **Sensors** and **Actuators**.
*   **Environment** *imposes* **Constraints** on **Agent**.
*   **Dynamics** *governs* the transition of **State**.
