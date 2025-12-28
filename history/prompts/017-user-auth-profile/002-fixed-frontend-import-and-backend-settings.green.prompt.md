---
id: "002"
title: "Fixed Frontend Import and Backend Settings"
stage: "green"
date: "2025-12-24"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "017-user-auth-profile"
branch: "017-user-auth-profile"
user: "Mutahir"
command: "fix errors"
labels: ["fix", "frontend", "backend", "pydantic", "better-auth"]
links:
  spec: "specs/017-user-auth-profile/spec.md"
files_yaml:
  - "Textbook/src/lib/auth-client.ts"
  - "backend/main.py"
---

# Prompt

Fix the frontend error `Module not found: Error: Can't resolve 'better-auth/react'` and the backend error `pydantic_core._pydantic_core.ValidationError: ... Extra inputs are not permitted`.

# Response

Fixed both issues:

1.  **Frontend**: Updated `Textbook/src/lib/auth-client.ts` to import `createAuthClient` from `@better-auth/client` instead of `better-auth/react`.
2.  **Backend**: Updated `backend/main.py` `Settings.Config` to include `extra = "ignore"`, allowing the application to ignore undefined environment variables like `BETTER_AUTH_SECRET` without crashing.
