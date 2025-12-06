# Data Model: Chapter 3 Concept Map

**Purpose**: Defines the key conceptual entities and their relationships within the chapter. This ensures consistency in terminology throughout the text.

## Concepts

### 1. The Humanoid
- **Definition**: A robot with a body plan roughly mimicking human morphology (head, torso, two arms, two legs/mobility).
- **Attributes**:
  - `Degrees of Freedom (DoF)`: Total number of movable joints.
  - `Height/Weight`: Anthropomorphic dimensions.
  - `Power Source`: Battery vs. Tethered vs. ICE (mostly obsolete).

### 2. Locomotion Type
- **Type: Bipedal**
  - *Description*: Walks on two legs.
  - *Pros*: All-terrain access (stairs).
  - *Cons*: Unstable, energy inefficient.
- **Type: Wheeled-Biped**
  - *Description*: Wheels at the end of legs (e.g., Handle, Eve).
  - *Pros*: Speed, efficiency on flat ground.
  - *Cons*: Limited on stairs/rough terrain.

### 3. Actuation Technology
- **Type: High-Ratio Geared (Electric)**
  - *Used by*: ASIMO, older industrial arms.
  - *Trait*: Stiff, precise, easily damaged by impact.
- **Type: Series Elastic / Quasi-Direct Drive (QDD)**
  - *Used by*: MIT Cheetah, Unitree, Modern Humanoids.
  - *Trait*: Compliant, back-drivable, robust to impact, dynamic.
- **Type: Hydraulic**
  - *Used by*: Old Atlas.
  - *Trait*: High power density, messy, loud, inefficient.

### 4. Control Paradigm
- **Type: ZMP (Zero Moment Point)**
  - *Era*: 2000-2015.
  - *Trait*: Flat-footed walking, very stable, slow.
- **Type: Model Predictive Control (MPC)**
  - *Era*: 2015-Present.
  - *Trait*: Dynamic, optimization-based, requires accurate models.
- **Type: Reinforcement Learning (RL)**
  - *Era*: 2020-Present.
  - *Trait*: Robust, learns from simulation, hard to interpret.

## Relationships

- **Actuation** determines **Control Paradigm** suitability (e.g., QDD allows for better force control needed for RL).
- **Locomotion** dictates **Application** (e.g., Wheeled for Warehouse flat floors, Bipedal for Home stairs).
