# Implementation Plan: User Authentication and Profile

**Branch**: `017-user-auth-profile` | **Date**: 2025-12-24 | **Spec**: [specs/017-user-auth-profile/spec.md](./spec.md)
**Input**: Feature specification from `specs/017-user-auth-profile/spec.md`

## Summary

Implement a secure, hybrid authentication system using **Better-Auth** (Node.js) alongside the existing Python backend and Docusaurus frontend. The system mandates a **two-step onboarding flow**: users sign up via Better-Auth, then are immediately redirected to a mandatory "Complete Profile" page to capture `python_proficiency` and `developer_role` before accessing any protected resources (RAG Chatbot). All data is persisted in a shared **Neon Postgres** database.

## Technical Context

**Language/Version**: Python 3.11+ (Backend), Node.js v20+ (Auth Service), TypeScript (Frontend/Auth)
**Primary Dependencies**: 
- Backend: `fastapi`, `asyncpg`, `pydantic`
- Auth Service: `better-auth`, `hono` (or express), `pg`
- Frontend: `docusaurus`, `@better-auth/client`, `react`
**Storage**: Neon Serverless Postgres (Shared)
**Testing**: `pytest` (Backend), `jest` (Auth/Frontend)
**Target Platform**: Vercel (Frontend/Auth), Railway/Render (Backend) or Unified Docker
**Project Type**: Full Stack (Python API + Node.js Auth + React SSG)
**Performance Goals**: <100ms profile context retrieval
**Constraints**: Strict TDD, Better-Auth exclusivity, No new auth database (shared schema)

## Constitution Check

*GATE: Passed Phase 1 design.*

- **Accuracy**: N/A (Functional feature).
- **Clarity**: Code and docs must be clear.
- **Reproducibility**: `package.json` and `requirements.txt` will be updated.
- **Rigor**: Using industry-standard auth library (Better-Auth).
- **Integrity**: N/A.
- **AI Responsibility**: N/A.
- **Data Privacy & Security**: Passwords handled by Better-Auth (secure hashing). Profile data stored securely in Neon.
- **AI Ethics**: Personalization data used to improve AI response relevance.
- **Performance & Scalability**: Shared DB ensures immediate consistency.
- **Strict Test-Driven Development**: Plan includes specific test tasks for Auth and Profile flows.

## Project Structure

### Documentation (this feature)

```text
specs/017-user-auth-profile/
├── plan.md              # This file
├── research.md          # Technology decisions
├── data-model.md        # Database schema & ERD
├── quickstart.md        # Dev guide
├── contracts/           # API definitions
│   └── profile-api.yaml
└── tasks.md             # To be generated
```

### Source Code (repository root)

```text
auth-server/ (NEW)
├── src/
│   ├── index.ts         # Better-Auth server entry
│   └── db.ts            # Shared DB connection
├── package.json
└── tsconfig.json

backend/
├── src/
│   ├── api/
│   │   └── dependencies/
│   │       └── auth.py  # Session validation middleware
│   └── models/
│       └── profile.py   # UserProfile Pydantic model
└── tests/
    └── integration/
        └── test_auth_flow.py

Textbook/ (Frontend)
├── src/
│   ├── components/
│   │   ├── AuthProvider.tsx
│   │   └── ProtectedRoute.tsx
│   ├── pages/
│   │   ├── signin.tsx
│   │   ├── signup.tsx
│   │   └── onboarding.tsx
│   └── theme/           # Docusaurus swizzling if needed
```

**Structure Decision**: We are introducing a lightweight `auth-server` (Node.js) to host Better-Auth, as it is a TS-first library. This service shares the Neon DB with the existing Python `backend` and Docusaurus `Textbook` frontend, creating a unified data layer despite the split runtime.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| New `auth-server` service | Better-Auth is TS-only; Backend is Python. | Rewriting backend in Node is too costly; implementing custom Python auth violates "Better-Auth Exclusive" rule. |
