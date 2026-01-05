---
id: "003"
title: "fix-rag-cold-start-issue"
stage: "fix"
date: "2026-01-05"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "book_rag_agent"
branch: "main"
user: "Mutahir"
command: "fix bug"
labels: ["backend", "rag", "qdrant", "bugfix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "backend/main.py"
tests: []
---

## Prompt
Issue Description (Observed Behavior):
On the deployed HuggingFace app, the RAG chatbot does not retrieve book-related information even if the app is kept active or “warmed up” by sending multiple messages through the deployed UI. However, when a single message is sent from the local environment, the RAG immediately starts working, and after that, the deployed app also begins retrieving answers correctly. In short, warming the deployed app via its own UI does not initialize retrieval, but triggering the system once from local causes retrieval to start working everywhere. This behavior is consistent and repeatable.

## Response
Identified that the `QdrantClient` was lazily initialized and potentially failing silently or timing out during the initial connection in the deployed environment. Added an explicit warmup step in `backend/main.py`'s `lifespan` handler to initialize the Qdrant client and verify the connection (by listing collections) during application startup. This ensures the connection is ready before user requests are processed.
