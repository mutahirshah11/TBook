# Tasks: ROS 2 Architecture and Core Concepts

**Feature Branch**: `005-ros2-architecture`
**Feature Spec**: [specs/005-ros2-architecture/spec.md](specs/005-ros2-architecture/spec.md)

## Phase 1: Setup (Project Initialization)

**Goal**: Prepare the directory structure and assets for the new chapter.

- [x] T001 Create `part2` directory in `Textbook/docs/`
- [x] T002 Create `chapter5` directory in `Textbook/static/img/part2/`
- [x] T003 Create empty `chapter-5-ros2-architecture-and-core-concepts.mdx` in `Textbook/docs/part2/`

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Establish the core chapter structure and shared content foundations.

- [x] T004 Update `Textbook/sidebars.ts` to include the new Part 2 and Chapter 5
- [x] T005 Define Frontmatter and Introduction section in `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`

## Phase 3: Conceptual Understanding (User Story 1)

**Goal**: Explain the high-level architecture (DDS, RMW, Node Graph).
**Story**: [US1] Conceptual Understanding of ROS 2

- [x] T006 [US1] Draft "The ROS 2 Graph" section (Nodes, Discovery) in `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T007 [US1] Draft "Under the Hood: DDS & Middleware" section (RMW, DDS, QoS) in `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T008 [US1] Create and add "System Architecture Diagram" (User Code -> rclpy -> RMW -> DDS) to `Textbook/static/img/part2/chapter5/` and reference it in MDX
- [x] T009 [US1] Create and add "Node Graph Diagram" (Nodes, Topics, Connections) to `Textbook/static/img/part2/chapter5/` and reference it in MDX

## Phase 4: Basic Communication Implementation (User Story 2)

**Goal**: Implement Topic communication (Pub/Sub) with Python examples.
**Story**: [US2] Basic Communication Implementation

- [x] T010 [US2] Draft "Communication Patterns: Topics" section in `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T011 [US2] Create and add "Pub/Sub Flow Diagram" to `Textbook/static/img/part2/chapter5/` and reference it in MDX
- [x] T012 [US2] Implement and add Python code example for "Simple Publisher" to the MDX file
- [x] T013 [US2] Implement and add Python code example for "Simple Subscriber" to the MDX file
- [x] T014 [US2] Add "Performance & Scaling" subsection (briefly touching on characteristics) to the Topics section

## Phase 5: Synchronous and Asynchronous Tasks (User Story 3)

**Goal**: Explain Services and Actions for command/response and long-running tasks.
**Story**: [US3] Synchronous and Asynchronous Tasks

- [x] T015 [US3] Draft "Communication Patterns: Services" section (Client/Server) in `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T016 [US3] Create and add "Service Flow Diagram" to `Textbook/static/img/part2/chapter5/` and reference it in MDX
- [x] T017 [US3] Draft "Communication Patterns: Actions" section (Goal/Feedback/Result) in `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T018 [US3] Create and add "Action Flow Diagram" to `Textbook/static/img/part2/chapter5/` and reference it in MDX

## Phase 6: System Configuration and Orchestration (User Story 4)

**Goal**: Explain Parameters and Launch files.
**Story**: [US4] System Configuration and Orchestration

- [x] T019 [US4] Draft "Configuration: Parameters" section in `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T020 [US4] Draft "Orchestration: Launch Files" section in `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T021 [US4] Add Python Launch file code example to the MDX file

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Add advanced concepts (briefly), summary, and verify against checklist.

- [x] T022 Add "Advanced Concepts" section (SROS2, Bag files) to `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T023 Add "Summary & Checklist" section to `Textbook/docs/part2/chapter-5-ros2-architecture-and-core-concepts.mdx`
- [x] T024 Verify all links and image references in the MDX file
- [x] T025 Run Docusaurus build to verify rendering and fix any errors

## Dependencies

1. **Phase 1 & 2** MUST be completed first to establish the file structure.
2. **Phase 3** (Concepts) is a prerequisite for **Phase 4** (Topics).
3. **Phase 4** (Topics) is a logical prerequisite for **Phase 5** (Services/Actions).
4. **Phase 6** (Launch) relies on understanding Nodes (Phase 3) and Params.
5. **Phase 7** is the final review.

## Parallel Execution Examples

- **Designers**: Can start T008, T009, T011, T016, T018 (Diagram creation) in parallel with content drafting (T006, T010, T015).
- **Writers**: Can draft independent sections (e.g., T019/T020 for Config/Launch) while others work on Communication Patterns (T010/T015/T017), provided the file structure (T003) exists.

## Implementation Strategy

- **MVP**: Complete Phases 1, 2, and 3 (Basic setup + Conceptual understanding).
- **Incremental**: Add Phase 4 (Topics + Code) next.
- **Full Feature**: Complete Phases 5, 6, and 7.
