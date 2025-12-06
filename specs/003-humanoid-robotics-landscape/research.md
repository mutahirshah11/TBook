# Research: Chapter 3 - Humanoid Robotics Landscape

**Feature**: Overview of the Humanoid Robotics Landscape
**Status**: Complete

## Technical Context & Decisions

### 1. Taxonomy Definitions
**Decision**: We will categorize humanoids based on three primary dimensions: **Locomotion**, **Actuation**, and **Intelligence**.
**Rationale**: These are the fundamental engineering choices that define a robot's capabilities and limitations.
**Alternatives Considered**: Categorizing by "Use Case" (Service vs. Industrial). Rejected because use cases overlap (e.g., Optimus is targetting both), whereas engineering architecture is distinct.

### 2. Historical Scope
**Decision**: Focus on the "Modern Dynamic Era" (post-2010) but acknowledge the "Static Era" (Honda P-series/ASIMO) as the foundation.
**Rationale**: The book focuses on *Physical AI*, which is most relevant to dynamic walkers. ASIMO represents the ZMP (Zero Moment Point) era, which is less relevant to modern RL-based control but historically important.

### 3. Industry Selection
**Decision**: Highlight a mix of Research-roots and Commercial-roots companies.
**Selected Players**:
- **Boston Dynamics**: The benchmark for dynamics (Atlas).
- **Tesla**: The benchmark for manufacturing scale and end-to-end AI (Optimus).
- **Agility Robotics**: The leader in current commercial deployment (Digit).
- **Figure AI**: The leader in "General Purpose" narrative and speed of execution.
- **Unitree**: The leader in accessible/low-cost hardware.
- **1X / Sanctuary AI**: Mentioned for different approaches (wheels/geared vs. cable-driven/teleop).

### 4. Connection to Physical AI
**Decision**: Explicitly link hardware choices to the "Sim-to-Real gap".
**Rationale**: Chapter 1 & 2 introduced the gap. Chapter 3 must explain *why* the gap exists (e.g., gear backlash, hydraulic non-linearities).

## Implementation Strategy

- **Format**: MDX (Markdown with React components if needed, but likely pure Markdown for this chapter).
- **Location**: `docs/part1/chapter3-humanoid-landscape.mdx`
- **Assets**: Use placeholder images or open-source compatible diagrams for the "Taxonomy" section.
