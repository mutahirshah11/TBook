---
id: "005"
title: "handle-api-rate-limits-in-agent-client"
stage: "fix"
date: "2026-01-05"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "chat_api"
branch: "main"
user: "Mutahir"
command: "fix error message"
labels: ["backend", "error-handling", "agent-client", "gemini-api"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "backend/agent_client.py"
tests: []
---

## Prompt
still 429 Quota error is there its not been modified

## Response
The `agent_client.py` wrapper was catching the exception and returning a default error string containing the raw stack trace, bypassing the router-level check. Updated `backend/agent_client.py` (specifically `run_agent`, `run_agent_stream`, and `run_agent_stream_with_logging`) to intercept 429/Quota errors internally and return the user-friendly message: "⚠️ System Busy: High traffic detected. Please wait a moment before trying again."
