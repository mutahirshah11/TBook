---
id: 001
title: Create Book RAG Agent Spec
stage: spec
date: 2025-12-09
surface: agent
model: gemini-2.0-flash-exp
feature: book-rag-agent
branch: 016-book-rag-agent
user: Mutahir
command: /sp.specify
labels: ["spec", "rag", "agent"]
links:
  spec: specs/016-book-rag-agent/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/016-book-rag-agent/spec.md
 - specs/016-book-rag-agent/checklists/requirements.md
tests: []
---

## Prompt

.. now write the specs for thi s: Goal to Build the standalone RAG Agent using the OpenAI Agents SDK (Python version), using Gemini_API_KEY, AsyncOpenAI, OpenAIChatCompletionModel, and Qdrant. This iteration builds the brain of the chatbot. No UI. No backend routes. No ChatKit yet. Core Tasks Create Book Rag Chatbot Agent using the Syntax of OpenAI Agents SDK Add guardrails that the agent dosent reply anything expect the Questions related to the context of the book (say sorry please ask Question related to the book only ) and Streaming is done while giing the answer Based on OpenAI Agents SDK Use AsyncOpenAI with: API key = Gemini_API_KEY Model = OpenAIChatCompletionModel Fully asynchronous Add Retrieval Layer Connect to Qdrant Cloud Query top-N chunks (default N=3) Format chunks + metadata (chapter, section, heading) Add Support for User-Selected Text If selected_text exists → skip Qdrant retrieval If not → normal vector search Build Prompt Constructor Inject relevant context: SYSTEM: You are an AI assistant for a technical book. Use ONLY the provided context to answer. Build the final user prompt: context chunks user query metadata blocks Add Logging and Debugging Utilities Retrieved chunk IDs Similarity scores Raw context sent to LLM Final Testing Run sample queries in Python shell Test with and without selected-text Validate metadata in answers Outcome : A fully working RAG AI agent that accepts queries and returns answers within in the books context — but not connected to any UI or backend yet.

## Response snapshot

Created feature branch `016-book-rag-agent`, wrote specification, and validated with checklist.

## Outcome

- ✅ Impact: Created initial spec for Book RAG Agent.
- 🧪 Tests: N/A
- 📁 Files: Created spec.md and checklist.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Spec created successfully based on detailed user requirements.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
