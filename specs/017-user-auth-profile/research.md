# Phase 0: Research & Technical Decisions

## 1. Technical Context & Unknowns

**Unknowns:**
- **Integration with Python Backend**: Better-Auth is a TypeScript library. The existing backend is Python (FastAPI).
- **Docusaurus Integration**: Docusaurus is a Static Site Generator (SSG) with client-side React. Better-Auth Client needs to run in the browser.
- **Session Validation**: Python backend needs to validate sessions created by Better-Auth (Node.js).

## 2. Research Findings

### Decision 1: Auth Architecture (Hybrid)
**Context**: We must use Better-Auth (TS) but have a Python backend.
**Decision**: Deploy a lightweight **Node.js Auth Proxy** (or Server) next to the Python backend.
- **Node.js Service**: Runs `better-auth` server. Connects to the **same Neon Postgres** database.
- **Python Backend**: Connects to the same Neon Postgres database. Reads the `user` and `session` tables directly to validate requests (or validates JWTs if we configure stateless).
- **Frontend (Docusaurus)**: Uses `@better-auth/client` to talk to the Node.js Auth Service for login/signup.

**Rationale**:
- Better-Auth runs best in Node.js. Porting its logic to Python is error-prone.
- Shared Database allows Python to see the sessions created by Node.js immediately.
- Zero duplication of user data.

### Decision 2: Docusaurus Integration
**Context**: Docusaurus is primarily static.
**Decision**: Use `docusaurus-plugin-better-auth` (if exists) or manual Client Integration.
- **Approach**: Wrap the Docusaurus root in a `<AuthProvider>` context.
- **Protection**: Create a standard `ProtectedRoute` component that checks `auth.session`. If null/loading, redirect.

### Decision 3: Middleware & Strict Redirect
**Context**: "Strict Profile Enforcement".
**Decision**:
- **Frontend**: `ProtectedRoute` checks `user.profile_complete` flag.
- **Backend (Python)**: Middleware that checks `user_profile` table for the current user. If missing, return `403 Profile Incomplete`.

### Decision 4: Database Schema
**Context**: Neon Postgres.
**Decision**:
- Table `user`, `session`, `account`, `verification` (Standard Better-Auth).
- Table `user_profile`:
    - `user_id` (FK to `user.id`)
    - `python_proficiency` (ENUM)
    - `developer_role` (ENUM)
    - `onboarding_completed` (BOOLEAN, computed or stored)

## 3. Selected Tech Stack
- **Auth Server**: Node.js + Hono (or Express) + Better-Auth.
- **Main Backend**: Python + FastAPI (Existing).
- **Database**: Neon Postgres (Shared).
- **Frontend**: Docusaurus + React + @better-auth/client.
