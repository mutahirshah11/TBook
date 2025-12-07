# Research: ROS 2 Architecture and Core Concepts

**Feature**: ROS 2 Architecture Chapter
**Date**: 2025-12-06

## Decisions & Rationale

### 1. Code Example Language
- **Decision**: Python
- **Rationale**: Lower barrier to entry for explaining architectural concepts compared to C++. Matches the "Foundation" nature of the chapter.
- **Alternatives**: C++ (rejected for this specific chapter to reduce cognitive load, though essential for later chapters).

### 2. Diagramming Strategy
- **Decision**: Use standard ROS 2 architectural layers: User Code -> rclpy -> rcl -> RMW -> DDS.
- **Rationale**: Accurately reflects the abstraction stack.
- **Alternatives**: Simplified "black box" view (rejected because the goal is to explain the *architecture*).

### 3. Content Structure (Part 2)
- **Decision**: Create `part2` folder for "Embodied AI & ROS 2 Foundations".
- **Rationale**: Logically separates basic theory (Part 1) from practical ROS 2 implementation (Part 2).

## Open Questions (Resolved)

- **SROS2 Scope**: Introductory only.
- **Bag Files**: Brief overview only.

## References

- [ROS 2 Design Architecture](https://design.ros2.org/articles/changes.html)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
