# Tasks: Sensor Systems

**Feature Branch**: `004-sensor-systems-chapter-4`
**Status**: Complete

## Dependencies

- Phase 1 (Setup) must be completed first.
- Phase 2 (Foundations) blocks all subsequent phases.
- Phase 3 (US1 - Individual Sensors) is the core content and must be done before Phase 4 and 5.
- Phase 4 (US2 - Applications) depends on Phase 3.
- Phase 5 (US3 - Fusion) depends on understanding individual sensors (Phase 3).
- Phase 6 (Polish) requires all content phases to be drafted.

## Phase 1: Setup (Project Initialization)

**Goal**: Initialize the chapter file and asset structure in the Docusaurus project.
**Tests**: Verification that files exist and site builds.

- [x] T001 Create chapter file with Frontmatter in `docs/part1/chapter4-sensor-systems.mdx`
- [x] T002 Create asset directory `static/img/chapter4/`
- [x] T003 Verify `sidebars.ts` includes the new chapter (or auto-generates correctly)

## Phase 2: Foundations (Blocking Prerequisites)

**Goal**: Establish the chapter introduction and scope (The "Perceptual Backbone").
**Tests**: Manual review against "Hook" and "Scope" in `contracts/outline.md`.

- [x] T004 Define Chapter Title, Metadata, and Introduction (Section 1) in `docs/part1/chapter4-sensor-systems.mdx`

## Phase 3: User Story 1 - Understanding Individual Sensor Capabilities (Priority: P1)

**Goal**: Describe the 4 major sensor classes (LiDAR, Camera, IMU, F/T).
**Story**: As a reader, I want to understand what each major sensor type measures and its physical principles.
**Independent Test**: Verify detailed descriptions for all 4 sensor types exist.

- [x] T005 [US1] Write Section 2.1: LiDAR (Principle, Data, Hardware) in `docs/part1/chapter4-sensor-systems.mdx`
- [x] T006 [US1] Write Section 2.2: Cameras (RGB/RGB-D, Principle, Data) in `docs/part1/chapter4-sensor-systems.mdx`
- [x] T007 [US1] Write Section 3.1: IMU (Principle, Drift, Criticality) in `docs/part1/chapter4-sensor-systems.mdx`
- [x] T008 [US1] Write Section 3.2: Force/Torque Sensors (Principle, Strain, Use Cases) in `docs/part1/chapter4-sensor-systems.mdx`
- [x] T009 [US1] [P] Create placeholder diagram for "Time of Flight" principle in `static/img/chapter4/tof_principle_placeholder.svg`

## Phase 4: User Story 2 - Real-world Sensor Applications and Hardware (Priority: P2)

**Goal**: Connect sensors to real-world tasks (SLAM, Manipulation) and industry examples.
**Story**: As a reader, I want to learn about high-level use cases and real-world hardware.
**Independent Test**: Verify each sensor section includes at least 2 use cases and specific hardware examples.

- [x] T010 [US2] Enrich LiDAR section with SLAM use case and industry examples (Velodyne, Ouster) in `docs/part1/chapter4-sensor-systems.mdx`
- [x] T011 [US2] Enrich Camera section with Object Rec/Depth use cases and examples (RealSense) in `docs/part1/chapter4-sensor-systems.mdx`
- [x] T012 [US2] Enrich IMU/FT sections with Balance/Manipulation use cases and examples (Xsens, ATI) in `docs/part1/chapter4-sensor-systems.mdx`

## Phase 5: User Story 3 - Multi-Sensor Fusion and Future Modules (Priority: P2)

**Goal**: Explain Sensor Fusion and link to future modules (ROS 2, Gazebo).
**Story**: As a reader, I want to understand how sensors are combined and how this prepares me for future modules.
**Independent Test**: Verify specific section on "Sensor Fusion" and "Summary/Bridge" exists.

- [x] T013 [US3] Write Section 4: Sensor Fusion (Concept, Uncertainty, Kalman Filter intuition) in `docs/part1/chapter4-sensor-systems.mdx`
- [x] T014 [US3] [P] Create placeholder diagram for "Fusion Pipeline" in `static/img/chapter4/fusion_pipeline_placeholder.svg`
- [x] T015 [US3] Write Section 5: Summary & Bridge to Simulation/ROS 2 in `docs/part1/chapter4-sensor-systems.mdx`

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Ensure readability, link integrity, and final polish.

- [x] T016 [P] Run spellcheck and grammar check on `docs/part1/chapter4-sensor-systems.mdx`
- [x] T017 Verify internal links to Ch 3 (Body) and Ch 1 (Physical AI)
- [x] T018 Check mobile responsiveness of diagrams/tables

## Parallel Execution Examples

- **Content Writing**: T005, T006, T007, T008 can be written in parallel by different authors.
- **Asset Creation**: T009 and T014 can run parallel to writing.

## Implementation Strategy

1.  **Setup**: Initialize file.
2.  **Core (US1)**: Draft the 4 sensor sections (Definitions + Principles).
3.  **Context (US2)**: Add the "Real World" layer (Hardware + Use Cases).
4.  **Integration (US3)**: Add the Fusion section and Bridge to next chapters.
5.  **Polish**: Final review.