# Tasks: Part 6 - Conversational Robotics

**Branch**: `013-part6-conversational-robotics` | **Spec**: [specs/013-part6-conversational-robotics/spec.md](spec.md) | **Plan**: [specs/013-part6-conversational-robotics/plan.md](plan.md)

## Phase 1: Setup
*Goal: Initialize the content structure and verify Python environment.*

- [ ] T001 Create chapter file structure in `Textbook/docs/part6/` (3 files)
- [ ] T002 Add "Part 6: Conversational Robotics" to `Textbook/sidebars.ts` with Chapters 21-23
- [ ] T003 Verify `ollama` and `vosk` pip packages
- [ ] T004 Verify `piper-tts` installation (may need external link check)

## Phase 2: Foundational Content (Prerequisites)
*Goal: Explain the AI stack.*

- [ ] T005 Write "Introduction to Conversational AI" in `chapter-21-llm-integration.md`
- [ ] T006 Write "Local vs Cloud LLMs" comparison (Privacy, Latency) in `chapter-21-llm-integration.md`
- [ ] T007 Write "The Action Server Pattern" explanation for LLMs in `chapter-21-llm-integration.md`

## Phase 3: User Story 1 - LLM Integration
*Goal: Connect ROS 2 to Ollama.*
*Story: [US1] Chapter 21: Integrating GPT Models*

- [ ] T008 [US1] Write "Defining the GenerateText Action" (.action file tutorial) in `chapter-21-llm-integration.md`
- [ ] T009 [US1] Create `ollama_node.py` code example (Action Server) in `chapter-21-llm-integration.md`
- [ ] T010 [US1] Write "Prompt Engineering for Robots" section (System prompts, JSON formatting) in `chapter-21-llm-integration.md`
- [ ] T011 [US1] Write "Testing the LLM Node" guide using `ros2 action send_goal` in `chapter-21-llm-integration.md`

## Phase 4: User Story 2 - Speech & NLU
*Goal: Hear and Speak.*
*Story: [US2] Chapter 22: Speech Recognition and NLU*

- [ ] T012 [US2] Write "Offline STT with Vosk" tutorial in `chapter-22-speech-nlu.md`
- [ ] T013 [US2] Create `speech_node.py` code example (Publishing strings) in `chapter-22-speech-nlu.md`
- [ ] T014 [US2] Write "Intent Classification with Grammars" guide in `chapter-22-speech-nlu.md`
- [ ] T015 [US2] Write "Text-to-Speech with Piper" section (Subscriber node) in `chapter-22-speech-nlu.md`

## Phase 5: User Story 3 - Multi-Modal Interaction
*Goal: See and Ground.*
*Story: [US3] Chapter 23: Multi-Modal Interaction*

- [ ] T016 [US3] Write "Zero-Shot VQA Concepts" section (CLIP, LlaVA) in `chapter-23-multimodal.md`
- [ ] T017 [US3] Create `vqa_node.py` code example using Hugging Face Transformers in `chapter-23-multimodal.md`
- [ ] T018 [US3] Write "The Grounding Pipeline" tutorial (Speech -> Intent -> Vision -> Action) in `chapter-23-multimodal.md`
- [ ] T019 [US3] Write "Handling Ambiguity" section (Robot asks clarification questions) in `chapter-23-multimodal.md`

## Phase 6: Polish & Review
*Goal: Ensure quality and consistency.*

- [ ] T020 Review all python code snippets for syntax correctness
- [ ] T021 Verify Flesch-Kincaid readability score (target grade 10-12)
- [ ] T022 Add "Next Steps" and "Further Reading" sections to all chapters

## Dependencies

- Phase 3 (LLM) MUST complete before Phase 5 (VQA relies on LLM reasoning).
- Phase 4 (Speech) is independent but required for the full pipeline in Phase 5.

## Implementation Strategy

1.  **Brain**: First, give the robot intelligence (LLM) in Phase 3.
2.  **Ears/Mouth**: Then, give it a voice (Speech) in Phase 4.
3.  **Eyes**: Finally, connect vision to language (VQA) in Phase 5.
