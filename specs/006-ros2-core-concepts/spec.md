# Feature Specification: ROS 2 Core Concepts (Chapters 6 & 7)

**Feature Branch**: `006-ros2-core-concepts`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "now make specs for chapter 6 and chapter 7 which is in part 2 chapter name for 6 : Nodes, topics, services, and actions chapter name for 7 : Building ROS 2 packages with Python"

## Clarifications

- Q: Which specific Python library for ROS 2 should be used for all code examples? → A: `rclpy` exclusively.
- Q: What level of prior ROS 2 knowledge is assumed for the reader? → A: No prior ROS 2 knowledge is assumed; chapters must explain all ROS 2 specific terms.
- Q: How should the ROS 2 environment setup be handled? → A: Assume setup is covered in an earlier chapter; these chapters focus on concepts and usage.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Master ROS 2 Communication Primitives (Priority: P1)

As a reader learning ROS 2, I need to understand the core communication concepts (Nodes, Topics, Services, Actions) so that I can architect robotic systems effectively.

**Why this priority**: These are the building blocks of any ROS 2 system. Without them, the reader cannot proceed to advanced topics.

**Independent Test**: Verify that the chapter clearly distinguishes between when to use a Topic, Service, or Action with concrete examples.

**Acceptance Scenarios**:

1. **Given** a reader is reading Chapter 6, **When** they finish the "Topics" section, **Then** they can explain the publisher-subscriber pattern.
2. **Given** the "Services" section, **When** the reader completes it, **Then** they understand the synchronous request-response pattern.
3. **Given** the "Actions" section, **When** the reader completes it, **Then** they understand the asynchronous goal-feedback-result pattern.

---

### User Story 2 - Build ROS 2 Python Packages (Priority: P1)

As a developer, I want to learn the standard workflow for creating, configuring, and building ROS 2 packages with Python so that I can distribute and manage my code.

**Why this priority**: Packaging is essential for code reuse and deployment in the ROS ecosystem.

**Independent Test**: Follow the chapter instructions to create a dummy package `ros2_python_pkg` and verify it builds with `colcon build`.

**Acceptance Scenarios**:

1. **Given** a clean ROS 2 workspace, **When** the reader runs the package creation commands, **Then** a valid directory structure with `setup.py` and `package.xml` is generated.
2. **Given** a created package, **When** the reader runs `colcon build`, **Then** the build succeeds without errors.
3. **Given** the build is successful, **When** the reader sources the overlay, **Then** they can run the package's executables.

---

### User Story 3 - Hands-on Coding Practice (Priority: P2)

As a student, I want to run executable Python examples for each concept (Pub/Sub, Service, Action) to verify my understanding through practice.

**Why this priority**: Theoretical knowledge must be reinforced with practical application in robotics.

**Independent Test**: Execute each provided code snippet in a standard ROS 2 environment and verify expected output.

**Acceptance Scenarios**:

1. **Given** the Publisher/Subscriber example code, **When** executed in two terminals, **Then** messages are successfully sent and received.
2. **Given** the Service example code, **When** a client sends a request, **Then** the server processes it and returns a response.

## Requirements *(mandatory)*

### Functional Requirements

#### Chapter 6: Nodes, Topics, Services, and Actions
- **FR-001**: The system (book) MUST provide a conceptual explanation of a ROS 2 **Node** and its lifecycle.
- **FR-002**: The system MUST explain **Topics** (Publisher/Subscriber) with a diagram and a Python code example.
- **FR-003**: The system MUST explain **Services** (Server/Client) with a diagram and a Python code example.
- **FR-004**: The system MUST explain **Actions** (Server/Client) with a diagram distinguishing it from Services (feedback mechanism).
- **FR-005**: The content MUST explain the command-line tools for debugging: `ros2 topic`, `ros2 service`, `ros2 action`, `ros2 node`.
- **FR-005.1**: Chapters 6 and 7 content MUST be organized within a `docs/part2` directory in the Docusaurus project structure.

#### Chapter 7: Building ROS 2 Packages with Python
- **FR-006**: The system MUST provide a step-by-step guide to creating a ROS 2 Python package using `ros2 pkg create`.
- **FR-007**: The system MUST explain the purpose and structure of `package.xml` (dependencies) and `setup.py` (entry points).
- **FR-008**: The system MUST explain the concept of a ROS 2 workspace (`src` directory) and the usage of `colcon build`.
- **FR-009**: The system MUST explain how to `source` the workspace setup files (`install/setup.bash`).
- **FR-010**: All Python code examples throughout Chapters 6 and 7 MUST exclusively use `rclpy`.

### Out of Scope

- Custom message, service, or action definition (creating `.msg`/`.srv`/`.action` files).
- Launch files (`ros2 launch`).
- The Parameter Server.
- C++ package creation and examples.
- Advanced build configuration (complex `CMakeLists.txt` or `setup.py` beyond basics).

### Key Entities

- **Concept Diagram**: Visual representation of data flow (e.g., One-to-Many for topics, One-to-One for services).
- **Code Snippet**: Executable Python block implementing a specific pattern.
- **Terminal Command**: Shell command for the user to execute.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of the provided code examples run successfully in the project's target ROS 2 environment (e.g., Humble).
- **SC-002**: A new user can create, build, and run a "Hello World" ROS 2 Python package in under 10 minutes following Chapter 7 instructions.
- **SC-003**: The content covers all 4 communication primitives (Nodes, Topics, Services, Actions).
- **SC-004**: The documentation build (`npm run build`) completes without errors including the new chapters.

### Assumptions

- The reader has a working ROS 2 installation (as covered in previous chapters/setup).
- The reader has basic familiarity with Python syntax.
- **No prior ROS 2 knowledge is assumed**; all ROS 2 specific terms and concepts must be introduced and explained within these chapters.
- **Environment setup is assumed to be covered in an earlier chapter**; these chapters focus on concepts and usage.

### Edge Cases

- **EC-001**: **User OS Differences**: How do the instructions handle Windows vs Linux paths and commands? (The book should prioritize Linux but provide Windows notes where critical, or reference the setup guide).
- **EC-002**: **Missing Dependencies**: What happens if the user attempts to build the package without having `colcon` or `rosdep` installed? (Instructions should include a prerequisite check or reference).
- **EC-003**: **Build Failures**: How does the content help users troubleshoot common build errors (e.g., missing `setup.py` entry)?