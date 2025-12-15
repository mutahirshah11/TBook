# Tasks: Book RAG Chatbot Agent

**Feature**: `016-book-rag-agent`
**Spec**: [specs/016-book-rag-agent/spec.md](spec.md)
**Plan**: [specs/016-book-rag-agent/plan.md](plan.md)
**Structure**: `src/agents/book_rag_agent/`

## Phase 1: Setup

**Purpose**: Initialize project structure and dependencies for OpenAI Agents SDK with Gemini.

- [x] T001 Initialize `pyproject.toml` with dependencies: `openai-agents-python`, `qdrant-client`, `cohere`, `async_openai`, `pydantic`.
- [x] T002 Create directory structure: `src/agents/book_rag_agent/` with `__init__.py`.
- [x] T003 Create `.env.example` with placeholders for `GEMINI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `COHERE_API_KEY`.
- [x] T004 Create `src/agents/book_rag_agent/utils.py` for shared logging configuration.
- [x] T005 Create `src/agents/book_rag_agent/settings.py` to handle environment variables and client initialization (Qdrant, Cohere, OpenAI/Gemini).

## Phase 2: Foundational

**Purpose**: Core data models and agent infrastructure.

- [x] T006 Define Pydantic models for `RelevanceOutput`, `ChunkMetadata`, `RetrievedChunk` in `src/agents/book_rag_agent/models.py`.
- [x] T007 Implement Qdrant client wrapper in `src/agents/book_rag_agent/settings.py` (ensure connection).
- [x] T008 Implement Cohere client wrapper in `src/agents/book_rag_agent/settings.py` (ensure connection).
- [x] T009 Implement `retrieve_book_context` function logic (without decorator yet) in `src/agents/book_rag_agent/tools.py`.
- [x] T010 [P] Create `scripts/test_rag_agent.py` skeleton with basic import tests.

## Phase 3: User Story 1 - Answer Questions using Book Context (P1)

**Goal**: Agent answers using book content via RAG.
**Independent Test**: `scripts/test_rag_agent.py` verifies tool calls and context usage.

- [x] T011 [US1] Wrap `retrieve_book_context` with `@function_tool` in `src/agents/book_rag_agent/tools.py`.
- [x] T012 [US1] Define `BookRAGAgent` in `src/agents/book_rag_agent/agent.py` using `Agent` class, instructions, and tools.
- [x] T013 [US1] Configure `BookRAGAgent` to use `Gemini` via `OpenAI` client (base_url adjustment) in `src/agents/book_rag_agent/agent.py`.
- [x] T014 [US1] Implement `ask_agent` interface function in `src/agents/book_rag_agent/interface.py` using `Runner.run()`.
- [x] T015 [US1] Create basic CLI entry point `scripts/rag_agent.py` that calls `ask_agent` and prints output.

## Phase 4: User Story 2 - User-Selected Context Override (P2)

**Goal**: Support manual context override.
**Independent Test**: Verify vector search is skipped when context is provided.

- [x] T016 [US2] Update `BookRAGAgent` instructions in `src/agents/book_rag_agent/agent.py` to prioritize provided context.
- [x] T017 [US2] Update `ask_agent` signature in `src/agents/book_rag_agent/interface.py` to accept `selected_text`.
- [x] T018 [US2] Update `scripts/rag_agent.py` CLI to support `--selected-text` argument.
- [x] T019 [US2] Test that `Runner` passes the context correctly in the input prompt.

## Phase 5: User Story 3 - Debugging and Transparency (P3)

**Goal**: Guardrails and Structured Logging.
**Independent Test**: Verify guardrail trips on irrelevant queries.

- [x] T020 [US3] Define `GuardrailAgent` in `src/agents/book_rag_agent/guardrails.py` with `RelevanceOutput`.
- [x] T021 [US3] Implement `relevance_guardrail` function using `@input_guardrail` in `src/agents/book_rag_agent/guardrails.py`.
- [x] T022 [US3] Attach `input_guardrails=[relevance_guardrail]` to `BookRAGAgent` in `src/agents/book_rag_agent/agent.py`.
- [x] T023 [US3] Add structured logging to `retrieve_book_context` in `src/agents/book_rag_agent/tools.py` (log chunks/scores).

## Phase 6: Polish

**Purpose**: Cleanup and final verification.

- [x] T024 [P] Run `uv sync` and verify all dependencies.
- [x] T025 Run full manual verification using `scripts/rag_agent.py --interactive`.
- [x] T026 Add final comments and documentation to `src/agents/book_rag_agent/`.

## Dependencies & Execution Order

1. **Setup (Phase 1)**: Must be first.
2. **Foundational (Phase 2)**: Depends on Setup.
3. **US1 (Phase 3)**: Depends on Foundational. Core RAG logic.
4. **US2 (Phase 4)**: Depends on US1. Adds context override.
5. **US3 (Phase 5)**: Depends on US1/US2. Adds safety/logging.
6. **Polish**: Final step.

## Implementation Strategy

1. **MVP**: Complete Phase 1, 2, and 3. Verify agent answers simple questions from the book.
2. **Enhanced**: Add Phase 4 (context override) and Phase 5 (Guardrails).
3. **Finalize**: Polish CLI and docs.
