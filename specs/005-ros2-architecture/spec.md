# Feature Specification: ROS 2 Architecture and Core Concepts

**Feature Branch**: `005-ros2-architecture`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Part 2 Chapter 5 ROS 2 architecture and core concepts"

## Clarifications

### Session 2025-12-06

- Q: Should out-of-scope topics be explicitly listed? → A: Explicitly list 1-2 key related topics that will *not* be covered (Option B).
- Q: What language should be used for code examples in the chapter? → A: Python (Option A).
- Q: Should the chapter discuss performance characteristics or scaling considerations? → A: Briefly touch on performance characteristics and scaling considerations (Option A).
- Q: Should the chapter include an introductory mention of ROS 2 security (SROS2)? → A: Include an introductory mention of SROS2 and its basic concepts (Option A).
- Q: How deeply should "Bag" files be covered? → A: Provide a brief overview of Bag functionality, its purpose, and common use cases (Option B).

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
-->

### User Story 1 - Conceptual Understanding of ROS 2 (Priority: P1)

A reader new to ROS 2 needs to understand the high-level architecture, including the node graph and underlying middleware, to grasp how robotic systems are composed.

**Why this priority**: This is the foundational knowledge required before writing any code.

**Independent Test**: Can be tested by reviewing the written content for clarity and accuracy regarding DDS, RMW, and the Node Graph.

**Acceptance Scenarios**:

1. **Given** a reader reading Chapter 5, **When** they complete the "Architecture" section, **Then** they can describe the role of DDS and the RMW layer.
2. **Given** a reader looking at the diagrams, **When** they study the Node Graph diagram, **Then** they can identify Nodes, Topics, and their connections.

---

### User Story 2 - Basic Communication Implementation (Priority: P1)

A reader wants to implement their first ROS 2 nodes to exchange data using the Publisher/Subscriber pattern.

**Why this priority**: Topics are the most fundamental communication mechanism in ROS 2.

**Independent Test**: Can be tested by running the provided example code (Publisher and Subscriber).

**Acceptance Scenarios**:

1. **Given** a configured ROS 2 environment, **When** the reader runs the example Publisher and Subscriber, **Then** the Subscriber receives and prints messages from the Publisher.

---

### User Story 3 - Synchronous and Asynchronous Tasks (Priority: P2)

A reader needs to understand when and how to use Services (synchronous) and Actions (long-running) to handle different types of robotic tasks.

**Why this priority**: Real robots require more than just data streaming; they need command interfaces and feedback loops.

**Independent Test**: Can be tested by verifying the explanations and examples distinguish clearly between Services and Actions.

**Acceptance Scenarios**:

1. **Given** a description of a robot task (e.g., "move to coordinate"), **When** the reader applies the chapter's guidance, **Then** they can correctly select "Action" over "Service" or "Topic".

---

### User Story 4 - System Configuration and Orchestration (Priority: P2)

A reader wants to configure node behavior without recompiling and start multiple nodes simultaneously.

**Why this priority**: Essential for managing complex robot systems with many nodes and varying environments.

**Independent Test**: Can be tested by verifying the Parameters and Launch system sections.

**Acceptance Scenarios**:

1. **Given** the launch file example, **When** executed, **Then** multiple nodes start with the specified parameters.

### Edge Cases

- **Reader uses an older ROS 2 version (e.g., Foxy)**: The chapter should briefly note any significant syntax differences (especially in Launch files) or explicitly state the target version (Humble/Jazzy) at the beginning.
- **Reader skips previous chapters**: The chapter should be self-contained regarding ROS 2 concepts, not relying on unexplained knowledge from Ch 1-4 unless explicitly referenced.

### Assumptions

- The reader has a basic understanding of Python and C++ syntax.
- The reader has installed a ROS 2 environment (as covered in previous setup chapters).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The chapter MUST explain the ROS 2 Node concept, emphasizing it as the fundamental unit of computation.
- **FR-002**: The chapter MUST detail the Publisher/Subscriber pattern (Topics) for asynchronous data streaming.
- **FR-003**: The chapter MUST explain the Service Client/Server pattern for synchronous request/response interactions.
- **FR-004**: The chapter MUST introduce Actions for long-running, preemptable tasks with feedback.
- **FR-005**: The chapter MUST explain the usage of Parameters for runtime configuration of nodes.
- **FR-006**: The chapter MUST introduce the Launch system (Python-based) for orchestrating multiple nodes.
- **FR-007**: The chapter MUST provide a high-level overview of the Data Distribution Service (DDS) and the Robot Middleware Interface (RMW).
- **FR-008**: The chapter MUST include at least one code example demonstrating a basic Publisher and Subscriber (in Python).
- **FR-009**: The chapter SHOULD briefly discuss the performance characteristics and scaling considerations of different communication patterns.
- **FR-010**: The chapter SHOULD include an introductory mention of ROS 2 security (SROS2) and its basic concepts.
- **FR-011**: The chapter SHOULD provide a brief overview of Bag file functionality, purpose, and common use cases.

### Out of Scope

- Detailed configuration of specific DDS vendors (FastDDS, CycloneDDS) beyond default settings.
- Advanced security features (SROS 2) and enclave configuration.

### Key Entities *(include if feature involves data)*

- **Node**: An executable that performs computation and communicates with other nodes.
- **Topic**: A named channel over which nodes exchange messages.
- **Message**: The data structure used for communication on Topics.
- **Service**: A pair of messages (Request/Response) for synchronous communication.
- **Action**: A set of messages (Goal, Result, Feedback) for long-running tasks.
- **Bag**: A format for recording and playing back message data (brief mention recommended).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The chapter includes at least 3 distinct diagrams (e.g., System Architecture, Pub/Sub flow, Service flow).
- **SC-002**: The provided code examples are complete and syntactically correct for ROS 2 Humble (or newer).
- **SC-003**: The content covers all 5 core communication concepts: Nodes, Topics, Services, Actions, Parameters.