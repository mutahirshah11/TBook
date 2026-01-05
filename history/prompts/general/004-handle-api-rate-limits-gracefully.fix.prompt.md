---
id: "004"
title: "handle-api-rate-limits-gracefully"
stage: "fix"
date: "2026-01-05"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "chat_api"
branch: "main"
user: "Mutahir"
command: "fix error message"
labels: ["backend", "error-handling", "ux", "gemini-api"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "backend/routers/chat.py"
tests: []
---

## Prompt
bhai kya kiya jae k jaab api limit reach hojae to is message ki jaaga koi or message dikhay k jiss sae professional lagay thora : 

Error: An error occurred while processing your request: Error code: 429 - ...

## Response
Updated `backend/routers/chat.py` to intercept 429/Quota Exceeded errors in both standard and streaming chat endpoints. Replaced the raw error stack trace with a user-friendly message: "⚠️ System Busy: High traffic detected. Please wait a moment before trying again."
