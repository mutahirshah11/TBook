# Tasks: Chapter 2 — Embodied AI and Physical Laws

**Branch**: `002-chapter-2-embodied-ai`
**Status**: Completed
**Feature**: Chapter 2 - Embodied AI and Physical Laws

## Implementation Phases

### Phase 1: Setup
**Goal**: Initialize the Docusaurus environment and file structure for Chapter 2.
- [x] T001 Create `chapter2-embodied-ai.mdx` file in `Textbook/docs/part1/` with front matter
- [x] T002 Create `img/chapter2/` directory in `Textbook/static/`
- [x] T003 Verify sidebar configuration in `Textbook/sidebars.ts` to include the new chapter

### Phase 2: Foundational Content (Blocking)
**Goal**: Establish the core structure and introduction of the chapter.
- [x] T004 Write "Introduction: The Brain in the Jar" section (Reality Gap, Digital vs Physical AI) in `Textbook/docs/part1/chapter2-embodied-ai.mdx`
- [x] T005 [P] Create/Find placeholder diagram for "Digital vs Physical AI" comparison in `Textbook/static/img/chapter2/digital-vs-physical.png`

### Phase 3: User Story 1 - The Reality Gap & Embodiment (Priority: P1)
**Goal**: Define Embodiment and explain why physical AI is different.
**Independent Test**: Reader can explain why ChatGPT cannot drive a car (Reality Gap).
- [x] T006 [US1] Write "What is Embodiment?" section (Definition, Morphological Computation) in `Textbook/docs/part1/chapter2-embodied-ai.mdx`
- [x] T007 [P] [US1] Create interactive component (or static diagram fallback) for "Software vs Hardware" dependency in `Textbook/src/components/Chapter2/EmbodimentSlider.tsx` (optional, or use static image)
- [x] T008 [US1] Integrate Embodiment section with Intro, ensuring "Interwoven Recap" from Ch1 in `Textbook/docs/part1/chapter2-embodied-ai.mdx`

### Phase 4: User Story 2 - Physics & Constraints (Priority: P1)
**Goal**: Introduce Physical Laws, Dynamics, Energy, and Uncertainty.
**Independent Test**: Reader can list the 3 pillars (Dynamics, Energy, Uncertainty).
- [x] T009 [US2] Write "The Laws of Physics" section (Gravity, Friction, Inertia) in `Textbook/docs/part1/chapter2-embodied-ai.mdx`
- [x] T010 [US2] Write "Dynamics: The Language of Motion" section (State $x$, Control $u$, Function $f$) in `Textbook/docs/part1/chapter2-embodied-ai.mdx`
- [x] T011 [P] [US2] Create/Find diagram for "The Feedback Loop" ($x \to y \to u \to f \to x'$) in `Textbook/static/img/chapter2/feedback-loop.png`
- [x] T012 [US2] Write "Energy: The Currency of Action" section (Thermodynamics, Efficiency) in `Textbook/docs/part1/chapter2-embodied-ai.mdx`
- [x] T013 [US2] Write "Uncertainty: The Fog of War" section (Noise types) in `Textbook/docs/part1/chapter2-embodied-ai.mdx`

### Phase 5: Polish & Cross-Cutting
**Goal**: Review, Refine, and Bridge to Ch3.
- [x] T014 Write "Conclusion & Roadmap" section with explicit bridge to Ch3 in `Textbook/docs/part1/chapter2-embodied-ai.mdx`
- [x] T015 [P] Create "Roadmap Flowchart" diagram for Part 1 in `Textbook/static/img/chapter2/part1-roadmap.png`
- [x] T016 Review content against "Light Formalism" constraint (ensure no heavy calculus)
- [x] T017 Run Docusaurus build `npm run build` to verify links and images

## Dependencies

1. **Phase 1** must be completed first to establish files.
2. **Phase 2** sets the tone and context.
3. **Phase 3** (US1) defines the *Agent*.
4. **Phase 4** (US2) defines the *Environment* and *Rules*.
5. **Phase 5** bridges to the next chapter.

## Parallel Execution Examples

- **Diagram Creation**: T005, T011, T015 can be done by a designer/parallel process while text is drafted.
- **Interactive Components**: T007 (React component) can be built while the text for T006 is being written.
