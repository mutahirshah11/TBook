---
description: "Task list for ROS 2 Core Concepts (Chapters 6 & 7)"
---

# Tasks: ROS 2 Core Concepts (Chapters 6 & 7)

**Input**: Design documents from `/specs/006-ros2-core-concepts/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/outline.md, quickstart.md

**Tests**: This feature is Documentation + Example Code. "Tests" here refer to verifying the provided code examples run correctly.

**Organization**: Tasks are grouped by user story to enable independent implementation and verification of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: US1, US2, US3
- **Path**: `docs/part2/` for documentation, `~/verify_ws/src/` context for examples

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Prepare the Docusaurus project structure

- [x] T001 Create `docs/part2` directory
- [x] T002 [P] Update `sidebars.ts` to include "Part 2: ROS 2 Core Concepts" category

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: N/A (Setup phase covers all dependencies for documentation)

---

## Phase 3: User Story 1 - Master ROS 2 Communication Primitives (Priority: P1) 🎯 MVP

**Goal**: Reader understands Nodes, Topics, Services, Actions via Chapter 6

**Independent Test**: Review Chapter 6 content and verify code snippets match `data-model.md` logic

### Implementation for User Story 1

- [x] T003 [US1] Create file `docs/part2/chapter-6-nodes-topics-services-actions.md` with metadata
- [x] T004 [US1] Write "Introduction to the ROS Graph" and "Nodes" section in `docs/part2/chapter-6-nodes-topics-services-actions.md`
- [x] T005 [US1] Write "Topics" section with `SimplePublisher` and `SimpleSubscriber` code examples in `docs/part2/chapter-6-nodes-topics-services-actions.md`
- [x] T006 [US1] Write "Services" section with `SimpleServiceServer` and `SimpleServiceClient` code examples in `docs/part2/chapter-6-nodes-topics-services-actions.md`
- [x] T007 [US1] Write "Actions" section (conceptual + diagram) in `docs/part2/chapter-6-nodes-topics-services-actions.md`
- [x] T008 [US1] Write "Introspection Tools" section covering `ros2` CLI commands in `docs/part2/chapter-6-nodes-topics-services-actions.md`

**Checkpoint**: Chapter 6 complete. Reader can understand communication patterns.

---

## Phase 4: User Story 2 - Build ROS 2 Python Packages (Priority: P1)

**Goal**: Reader can create and build a ROS 2 Python package via Chapter 7

**Independent Test**: Follow Chapter 7 steps to build a package

### Implementation for User Story 2

- [x] T009 [US2] Create file `docs/part2/chapter-7-building-packages.md` with metadata
- [x] T010 [US2] Write "The ROS 2 Workspace" section in `docs/part2/chapter-7-building-packages.md`
- [x] T011 [US2] Write "Creating a Python Package" section (using `ros2 pkg create`) in `docs/part2/chapter-7-building-packages.md`
- [x] T012 [US2] Write "Managing Dependencies" section (`package.xml`) in `docs/part2/chapter-7-building-packages.md`
- [x] T013 [US2] Write "Defining Entry Points" section (`setup.py`) in `docs/part2/chapter-7-building-packages.md`
- [x] T014 [US2] Write "Building and Running" section (`colcon build`, `source`) in `docs/part2/chapter-7-building-packages.md`

**Checkpoint**: Chapter 7 complete. Reader can build packages.

---

## Phase 5: User Story 3 - Hands-on Coding Practice (Priority: P2)

**Goal**: Verify code examples are correct and copy-pasteable

**Independent Test**: Run `quickstart.md` instructions

### Implementation for User Story 3

- [x] T015 [US3] Verify `SimplePublisher` code in Chapter 6 matches `data-model.md`
- [x] T016 [US3] Verify `SimpleSubscriber` code in Chapter 6 matches `data-model.md`
- [x] T017 [US3] Verify `SimpleServiceServer` code in Chapter 6 matches `data-model.md`
- [x] T018 [US3] Verify `SimpleServiceClient` code in Chapter 6 matches `data-model.md`
- [x] T019 [US3] [P] Add "Try it yourself" callouts in Chapter 6 linking to Chapter 7 workflow

**Checkpoint**: All code examples validated.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements and verification

- [x] T020 [P] Verify internal links between Chapter 6 and 7
- [x] T021 [P] Check Docusaurus build `npm run build`
- [x] T022 [P] Verify spelling and grammar

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)**: No dependencies
- **User Story 1 (Phase 3)**: Depends on Setup
- **User Story 2 (Phase 4)**: Depends on Setup (Independent of US1)
- **User Story 3 (Phase 5)**: Depends on US1 and US2 (Needs content to verify)

### Implementation Strategy
1. Setup project structure
2. Write Chapter 6 (Concepts & Code)
3. Write Chapter 7 (Build System)
4. Review/Polish

### Parallel Opportunities
- Chapter 6 and Chapter 7 can be written in parallel after Setup.
- Review tasks can happen alongside writing.
