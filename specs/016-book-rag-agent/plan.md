# Implementation Plan: Book RAG Chatbot Agent

**Branch**: `016-book-rag-agent` | **Date**: 2025-12-10 | **Spec**: [specs/016-book-rag-agent/spec.md](spec.md)
**Input**: Feature specification from `/specs/016-book-rag-agent/spec.md`

## Summary

Build a structured RAG Chatbot Agent using the **OpenAI Agents SDK** (Python), powered by **Gemini** (via OpenAI compatibility) for generation and **Qdrant** + **Cohere** for retrieval. The agent will support context retrieval, user-selected text overrides, and input guardrails to ensure relevance. Code will be organized in `src/agents/book_rag_agent/`.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: `openai-agents-python`, `qdrant-client`, `cohere`, `async_openai`, `pydantic`
**Storage**: Qdrant (Vector Store)
**Testing**: `pytest`
**Target Platform**: CLI / Python Script
**Project Type**: Single Script / Module
**Performance Goals**: <3s latency for answers
**Constraints**: Use OpenAI Agents SDK syntax; Use Gemini API; Separate folder for Agent.
**Scale/Scope**: Single agent, book-sized knowledge base.

## Constitution Check

- [x] **Accuracy**: RAG ensures accuracy from book content.
- [x] **Clarity**: Code structured in `src/agents/`.
- [x] **Reproducibility**: `uv` for dependencies.
- [x] **Rigor**: Guardrails and structured outputs used.
- [x] **Integrity**: Original implementation.
- [x] **AI Responsibility**: Guardrails prevent out-of-scope answers.
- [x] **Data Privacy**: No user data persistence; secure API key usage.
- [x] **AI Ethics**: Guardrails reduce misuse.
- [x] **Performance**: Async I/O used.
- [x] **Strict Test-Driven Development**: Plan includes testing tasks.

## Project Structure

### Documentation (this feature)

```text
specs/016-book-rag-agent/
├── plan.md              # This file
├── research.md          # Implementation decisions
├── data-model.md        # Agent & Data structures
├── quickstart.md        # Usage guide
├── contracts/           # API definitions
└── tasks.md             # TDD tasks
```

### Source Code

```text
src/
└── agents/
    └── book_rag_agent/
        ├── __init__.py
        ├── agent.py         # Main Agent definition
        ├── tools.py         # @function_tool definitions
        ├── guardrails.py    # Guardrail logic
        └── utils.py         # Helpers
scripts/
├── rag_agent.py             # CLI Entrypoint
└── test_rag_agent.py        # Tests
```

**Structure Decision**: Separate `src/agents/book_rag_agent` folder as requested for clean organization.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | | |
