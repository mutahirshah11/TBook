---
id: "008"
title: "fix-cohere-rate-limit-and-logging"
stage: "fix"
date: "2026-01-05"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "book_rag_agent"
branch: "main"
user: "Mutahir"
command: "fix cohere 429"
labels: ["backend", "cohere", "rate-limit", "logging"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "backend/main.py"
  - "backend/agents/book_rag_agent/tools.py"
tests: []
---

## Prompt
ye still masla karta hai ... (Logs show Cohere 429 Too Many Requests and OpenAI 401)

## Response
The aggressive warmup strategy was consuming the free-tier Cohere quota (429 error).
1. Modified `backend/main.py` to **skip** the actual Cohere API call during warmup (only initializing the client).
2. Updated `backend/agents/book_rag_agent/tools.py` to gracefully handle `TooManyRequests` exceptions, returning a polite "System Busy" message instead of crashing.
3. Fixed unreachable logging code in `tools.py`.
