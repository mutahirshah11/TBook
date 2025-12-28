# Tasks: User Authentication and Profile

**Feature Branch**: `017-user-auth-profile`
**Spec**: [specs/017-user-auth-profile/spec.md](spec.md)
**Plan**: [specs/017-user-auth-profile/plan.md](plan.md)

## Phase 1: Setup & Infrastructure (Hybrid Auth)

**Goal**: Initialize the Node.js Auth Server, configure shared database, and solve the cross-origin cookie sharing challenge.

- [x] T001 Initialize `auth-server` directory with Node.js, TypeScript, and `better-auth` dependencies in `auth-server/package.json`
- [x] T002 Create shared database migration for `user_profile` table (and Better-Auth tables) in `backend/scripts/migrations/001_auth_profile.sql`
- [x] T003 Configure `auth-server` to use Neon Postgres and expose Better-Auth API at port 4000 in `auth-server/src/index.ts`
- [x] T004 **CRITICAL**: Configure `auth-server` CORS to allow requests from Docusaurus (port 3000) and cookie domain for sharing with Python Backend (port 8000) in `auth-server/src/index.ts`
- [x] T005 [P] Install `@better-auth/client` and dependencies in Docusaurus project `Textbook/package.json`
- [x] T006 [P] Install `asyncpg` and `pydantic` in Python backend `backend/requirements.txt` if not present

## Phase 2: Foundational Components

**Goal**: Establish the base layers for session validation on both Frontend (Client) and Backend (API).

- [x] T007 Create `AuthProvider` component in Docusaurus to wrap app with Better-Auth client in `Textbook/src/components/Auth/AuthProvider.tsx`
- [x] T008 Integrate `AuthProvider` into Docusaurus root layout in `Textbook/src/theme/Root.js` (or `tsx`)
- [x] T009 [P] Create Python dependency `get_current_user` to validate session token from shared cookie against DB in `backend/src/api/dependencies/auth.py`
- [x] T010 Create `UserProfile` Pydantic model matching the DB schema in `backend/src/models/profile.py`

## Phase 3: User Story 1 - Signup & Onboarding (Priority P1)

**Goal**: User can create an account and is forced to complete their profile (Python Proficiency, Role) before doing anything else.

**Tests**:
- [x] T011 [US1] Create integration test for `POST /api/profile/onboarding` verifying immutability and data persistence in `backend/tests/integration/test_onboarding.py`

**Implementation**:
- [x] T012 [US1] Create Signup Page in Docusaurus using Better-Auth client methods in `Textbook/src/pages/signup.tsx`
- [x] T013 [US1] Create Onboarding Page with form for Proficiency/Role in `Textbook/src/pages/onboarding.tsx`
- [x] T014 [US1] Implement `POST /api/profile/onboarding` endpoint in Python backend to save profile data in `backend/routers/profile.py`
- [x] T015 [US1] Add logic to `onboarding.tsx` to redirect to `/dashboard` upon success in `Textbook/src/pages/onboarding.tsx`
- [x] T016 [US1] Update `signup.tsx` to redirect to `/onboarding` immediately after successful registration in `Textbook/src/pages/signup.tsx`

## Phase 4: User Story 2 - Signin & Session Enforcement (Priority P1)

**Goal**: Users can sign in, and "partial" users (valid auth, missing profile) are strictly redirected to onboarding.

- [x] T017 [US2] Create test ensuring users with `is_onboarded=False` are rejected from protected APIs in `backend/tests/integration/test_enforcement.py`

**Implementation**:
- [x] T018 [US2] Create Signin Page in Docusaurus in `Textbook/src/pages/signin.tsx`
- [x] T019 [US2] Create `ProtectedRoute` component that checks `auth.session` AND `user.is_onboarded` in `Textbook/src/components/Auth/ProtectedRoute.tsx`
- [x] T020 [US2] Implement client-side redirect: If `session` exists but `!is_onboarded`, force go to `/onboarding` in `Textbook/src/components/Auth/AuthProvider.tsx` (Global check)
- [x] T021 [US2] Implement Dashboard page (Protected) to verify successful flow in `Textbook/src/pages/dashboard.tsx`
- [x] T022 [US2] Update Python auth dependency to raise `403 Profile Incomplete` if `user_profile` is missing or incomplete in `backend/src/api/dependencies/auth.py`

## Phase 5: User Story 3 - RAG Chatbot Access Control (Priority P2)

**Goal**: Lock down the Chatbot so only fully authenticated+onboarded users can use it.

**Implementation**:
- [x] T023 [US3] Wrap Chatbot UI route with `ProtectedRoute` in `Textbook/src/pages/chatbot.tsx` (or wherever Chatbot lives)
- [x] T024 [US3] Apply `get_current_user` dependency to all Chatbot API routes in `backend/routers/chat.py`
- [x] T025 [US3] Verify Guest users can still access public book chapters (No changes needed, but verify) in `Textbook/src/pages/index.tsx`

## Phase 6: User Story 4 - Personalization Context (Priority P3)

**Goal**: Inject user profile data into the RAG context.

**Implementation**:
- [x] T026 [US4] Modify `ChatRequest` or context builder to accept/retrieve `user_profile` data in `backend/src/services/rag_service.py`
- [x] T027 [US4] Update RAG system prompt to include "User Proficiency: {level}" and "Role: {role}" instructions in `backend/agents/book_rag_agent/prompts.py` (or similar)

## Phase 7: Polish & Cross-Cutting

- [x] T028 Add "Sign Out" button to Navbar (conditional on auth state) in `Textbook/src/theme/NavbarItem/Component.tsx` (or standard Docusaurus config)
- [x] T029 Verify Cookie Security (HttpOnly, Secure, SameSite=Lax/None) across all 3 services
- [ ] T030 Run full E2E manual test: Signup -> Onboarding -> Dashboard -> Chatbot -> Signout

## Dependencies

- **Phase 1 -> Phase 2**: Infrastructure must be ready before creating components.
- **Phase 2 -> Phase 3**: Auth providers and DB models needed for Signup.
- **Phase 3 -> Phase 4**: Onboarding logic needed to enforce redirects.
- **Phase 4 -> Phase 5/6**: Strict enforcement needed before protecting specific features.

## Parallel Execution Opportunities

- **T005/T006**: Frontend and Backend dependencies can be installed in parallel.
- **T009/T010**: Backend Auth logic and Model creation are independent.
- **T012/T013**: Signup and Onboarding pages can be built simultaneously by different frontend devs.
