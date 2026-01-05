---
id: "006"
title: "fix-rag-connectivity-and-timeout"
stage: "fix"
date: "2026-01-05"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "book_rag_agent"
branch: "main"
user: "Mutahir"
command: "fix persistent rag issue"
labels: ["backend", "qdrant", "timeout", "networking"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "backend/agent_client.py"
  - "backend/agents/book_rag_agent/settings.py"
tests: []
---

## Prompt
Issue Description (Observed Behavior):
On the deployed HuggingFace app, the RAG chatbot does not retrieve book-related information... This issue is still there what do do ? what to check ?

## Response
The previous "warmup" fix confirmed connectivity but didn't solve the runtime retrieval failure.
Analysis points to two potential causes for the environment-specific failure:
1.  **Network Protocol Blocking:** Hugging Face might be blocking gRPC ports (default for Qdrant), causing the search to hang or fail.
2.  **Timeout:** The default 30s timeout might be insufficient for the first "cold" search on a serverless Qdrant instance.

Fixes Applied:
1.  **Forced HTTP:** Updated `backend/agents/book_rag_agent/settings.py` to set `prefer_grpc=False` in `QdrantClient`, ensuring it uses port 6333 (HTTP) which is more reliably open.
2.  **Increased Timeout:** Updated `backend/agent_client.py` to increase the default `AGENT_TIMEOUT` from 30s to 60s.
