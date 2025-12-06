# Content Structure: Chapter 2

**File**: `Textbook/docs/part1/chapter2-embodied-ai.mdx`

## Front Matter
```yaml
---
sidebar_position: 2
title: "Chapter 2: Embodied AI and Physical Laws"
description: "From digital brains to physical bodies: understanding constraints, dynamics, and energy."
---
```

## Section Outline

### 1. Introduction: The Brain in the Jar
*   **Goal**: Hook the reader. Contrast Digital AI vs. Physical AI.
*   **Content**:
    *   Analogy: "The Reality Gap".
    *   Case Study: Why ChatGPT can't drive a car yet.
    *   *Learning Outcome*: Understand why embodiment changes the problem scope.

### 2. What is Embodiment?
*   **Goal**: Define the core entity.
*   **Content**:
    *   Definition: "Intelligence requires a body."
    *   *Interactive Component*: Simple slider/toggle showing "Software" vs "Hardware" dependencies.
    *   Key Concept: The hardware *is* part of the intelligence (Morphological Computation).

### 3. The Laws of Physics (The Operating System)
*   **Goal**: Introduce the constraints.
*   **Content**:
    *   **Gravity**: The constant downward force.
    *   **Friction**: The resistance to motion.
    *   **Inertia**: The resistance to change ($F=ma$).
    *   **Collision**: The harsh reality of solid objects.

### 4. Dynamics: The Language of Motion
*   **Goal**: Light formalism.
*   **Content**:
    *   State ($x_t$): Where am I?
    *   Control ($u_t$): What am I doing?
    *   Function ($f$): What happens next? $x_{t+1} = f(x_t, u_t)$.
    *   *Diagram*: The Feedback Loop (State -> Sense -> Think -> Act -> Dynamics -> State).

### 5. Energy: The Currency of Action
*   **Goal**: Discuss efficiency.
*   **Content**:
    *   Digital bits are cheap; Physical Joules are expensive.
    *   Thermodynamics link: Heat dissipation.
    *   Constraint: Battery life vs. Performance.

### 6. Uncertainty: The Fog of War
*   **Goal**: Introduce noise.
*   **Content**:
    *   Sensors lie (Measurement noise).
    *   Motors slip (Actuation noise).
    *   The World changes (Environment noise).

### 7. Conclusion & Roadmap
*   **Goal**: Recap and Bridge.
*   **Content**:
    *   Summary: Embodiment = Constraints + Opportunities.
    *   Bridging: "Now that we know the rules ($f$), how do we control ($u$)? -> Chapter 3".
    *   *Visual Roadmap*: Flowchart of Part 1 chapters.
