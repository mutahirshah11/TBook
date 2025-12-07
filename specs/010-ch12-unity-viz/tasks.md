# Tasks: Chapter 12 - Unity for Robot Visualization

**Branch**: `010-ch12-unity-viz` | **Spec**: [specs/010-ch12-unity-viz/spec.md](spec.md) | **Plan**: [specs/010-ch12-unity-viz/plan.md](plan.md)

## Phase 1: Setup
*Goal: Initialize the chapter content structure and verify environment prerequisites.*

- [x] T001 Create chapter file structure in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T002 Add "Chapter 12: Unity Visualization" to `Textbook/sidebars.ts`
- [x] T003 Verify `com.unity.robotics.ros-tcp-connector` git URL is accessible
- [x] T004 Verify `ros-tcp-endpoint` repository is accessible

## Phase 2: Foundational Content (Prerequisites)
*Goal: Explain the architecture and guide the user through the installation of tools.*

- [x] T005 Write "Introduction to Robot Visualization" section (Why Unity? Digital Twin concept) in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T006 Write "Architecture Overview" section explaining TCP Handshake and Serialization in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T007 Write "Installing Unity Hub and Editor" guide (specifying 2022.3 LTS) in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T008 Write "Creating a URP Project" guide in `Textbook/docs/part3/chapter-12-unity-viz.md`

## Phase 3: User Story 1 - Setting Up the Environment
*Goal: Establish a working ROS2-Unity connection.*
*Story: [US1] Setting Up the Unity Development Environment*

- [x] T009 [US1] Write "Installing ROS-TCP-Connector" section (Package Manager Git URL) in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T010 [US1] Write "Setting up ROS-TCP-Endpoint in ROS2" section (Clone, Build, Run) in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T011 [US1] Write "Networking Configuration" section covering Host IP, WSL2 IP, and Windows Firewall rules in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T012 [US1] Write "Verifying the Connection" tutorial using the HUD status indicator in `Textbook/docs/part3/chapter-12-unity-viz.md`

## Phase 4: User Story 2 - Importing Robot Models
*Goal: Import a URDF file and visualize it with correct physics.*
*Story: [US2] Importing Robot Models into Unity*

- [x] T013 [US2] Write "Installing URDF Importer" section in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T014 [US2] Write "Importing a URDF" tutorial (using the Chapter 10 robot) in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T015 [US2] Write "Fixing Pink Materials" guide using the URP Render Pipeline Converter in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T016 [US2] Write "Understanding ArticulationBody" conceptual section (vs Rigidbody) in `Textbook/docs/part3/chapter-12-unity-viz.md`

## Phase 5: User Story 3 - ROS2-Unity Communication
*Goal: Exchange data between the simulator and ROS2 nodes.*
*Story: [US3] Establishing ROS2-Unity Communication*

- [x] T017 [US3] Write "Generating C# Message Structs" guide using the Robotics > Generate ROS Messages tool in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T018 [US3] Create `RosSubscriber.cs` code example for `cmd_vel` (Twist) messages in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T019 [US3] Create `RosPublisher.cs` code example for `joint_states` messages in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T020 [US3] Write "Testing Communication" guide using `ros2 topic pub` and `ros2 topic echo` in `Textbook/docs/part3/chapter-12-unity-viz.md`

## Phase 6: Polish & Review
*Goal: Ensure quality and add troubleshooting resources.*

- [x] T021 Write "Troubleshooting" section (Firewall, IP issues, Version Mismatch) in `Textbook/docs/part3/chapter-12-unity-viz.md`
- [x] T022 Review content against "Flesch-Kincaid grade 10-12" clarity standard
- [x] T023 Verify all links to external Unity/ROS documentation are valid
- [x] T024 Add screenshots placeholders (instructions for where to capture them) in `Textbook/docs/part3/chapter-12-unity-viz.md`

## Dependencies

- Phase 2 (Foundation) MUST complete before Phase 3, 4, 5.
- Phase 3 (Setup) MUST complete before Phase 5 (Communication).
- Phase 4 (Import) depends on Phase 3 (Setup) for package availability but can be drafted in parallel.
- Phase 5 (Communication) depends on Phase 3 (Setup).

## Implementation Strategy

1. **MVP**: Complete Phases 1, 2, and 3. This gives the reader a connected environment.
2. **Visuals**: Complete Phase 4. This gives the reader their robot in 3D.
3. **Interactivity**: Complete Phase 5. This makes the simulation "alive".
