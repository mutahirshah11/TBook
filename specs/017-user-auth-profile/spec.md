# Feature Specification: User Authentication and Profile

**Feature Branch**: `017-user-auth-profile`
**Created**: 2025-12-24
**Status**: Draft
**Input**: User description provided via prompt.

## Clarifications

### Session 2025-12-24
- Q: How should the additional profile data (proficiency, role) be submitted during the signup process? → A: **Two-Step (Onboarding)**: Create the auth user first, then immediately redirect to a "Complete Profile" page to save extra data.
- Q: How should the system handle a logged-in user who hasn't completed their profile? → A: **Strict (Global Redirect)**: Immediately redirect them to the "Complete Profile" page upon login/navigation. Block access to ALL protected routes until finished.
- Q: Should users be able to update their profile data (Proficiency, Role) after the initial signup? → A: **No, Immutable**: Data is collected once at signup and cannot be changed by the user.
- Q: Where should the user be redirected after successfully completing their profile? → A: **Redirect to Dashboard**: Take them to a central "Dashboard" landing page.
- Q: Which authentication methods should be supported in this feature? → A: **Email/Password + Social (Google/GitHub)**: Support both credentials and major social providers.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - User Signup and Onboarding (Priority: P1)

As a new user, I want to create an account and be guided to set up my profile so that I can access personalized features.

**Why this priority**: Foundation for all other personalized features. Without signup and profile data collection, personalization is impossible.

**Independent Test**: Can be tested by creating an account and verifying the redirection to the profile setup page, then completing the profile.

**Acceptance Scenarios**:

1. **Given** a guest user on the signup page, **When** I enter valid credentials (email/password), **Then** my account is created, I am authenticated, and immediately redirected to the "Complete Profile" page.
2. **Given** a user on the "Complete Profile" page, **When** I select my Python proficiency and Developer role and submit, **Then** my profile data is saved, and I am redirected to the dashboard.
3. **Given** a successful onboarding completion, **When** I check the database, **Then** a User record exists in Better-Auth tables and a corresponding UserProfile record exists with the correct data.

---

### User Story 2 - User Signin and Session Enforcement (Priority: P1)

As a registered user, I want to sign in and stay signed in, but I understand I must complete my profile to access the system.

**Why this priority**: Essential for user experience and enforcing data completeness.

**Independent Test**: Can be tested by signing in with a partial account (simulated) and verifying the redirect loop until completion.

**Acceptance Scenarios**:

1. **Given** a registered user with a **complete** profile, **When** I enter correct credentials, **Then** I am authenticated and redirected to the dashboard or previous page.
2. **Given** a registered user with an **incomplete** profile, **When** I enter correct credentials, **Then** I am authenticated but redirected immediately to the "Complete Profile" page.
3. **Given** a user with an incomplete profile, **When** I attempt to navigate to the Dashboard manually, **Then** I am automatically redirected back to the "Complete Profile" page.
4. **Given** an authenticated user, **When** I click "Sign Out", **Then** my session is terminated and I am redirected to the public area.

---

### User Story 3 - RAG Chatbot Access Control (Priority: P2)

As a system administrator, I want to ensure only authenticated users can use the RAG Chatbot so that we can manage resources and provide personalization.

**Why this priority**: Enforces the business rule that AI features are for registered users only.

**Independent Test**: Can be tested by accessing the chatbot route as a guest and as a logged-in user.

**Acceptance Scenarios**:

1. **Given** a guest (unauthenticated) user, **When** I attempt to access or send a message to the RAG Chatbot, **Then** I am denied access and prompted to log in.
2. **Given** an authenticated user, **When** I access the RAG Chatbot, **Then** the interface loads and I can send messages.
3. **Given** a guest user, **When** I browse public book chapters, **Then** I can read content without being asked to log in.

---

### User Story 4 - Personalization Context Availability (Priority: P3)

As a user using the Chatbot, I want the system to know my background so that answers are tailored to my skill level.

**Why this priority**: Delivers the core value proposition of the personalization feature.

**Independent Test**: Can be tested by verifying the backend retrieves the correct profile data when a chat session initiates.

**Acceptance Scenarios**:

1. **Given** an authenticated user with "Beginner" Python proficiency, **When** I start a chat session, **Then** the system retrieves my "Beginner" status to be used in the RAG context.
2. **Given** an authenticated user with "Advanced" Python proficiency, **When** I start a chat session, **Then** the system retrieves my "Advanced" status.

### Edge Cases

- **Duplicate Accounts**: User tries to sign up with an existing email -> Should show clear error.
- **Database Failure**: Database is down during signup -> Should show user-friendly error, not crash.
- **Missing Profile**: Existing user (if any) tries to use Chatbot without profile data -> Should prompt to complete profile (Migration scenario, though current scope assumes new users).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST use **Better-Auth** library as the exclusive authentication provider.
- **FR-002**: System MUST collect 'Python Proficiency' (Options: Beginner, Intermediate, Advanced) via a post-signup onboarding flow.
- **FR-003**: System MUST collect 'Developer Role' (Options: Frontend Developer, Backend Developer, Full Stack Developer, None) via a post-signup onboarding flow.
- **FR-004**: System MUST persist authentication and profile data in the **Neon Serverless Postgres** database.
- **FR-005**: System MUST use a relational database structure where user profiles are linked to Better-Auth user IDs.
- **FR-006**: System MUST allow public access to all book chapters (Markdown content) without authentication.
- **FR-007**: System MUST require valid authentication to access the RAG Chatbot interface and API.
- **FR-008**: System MUST make the authenticated user's profile data available to the RAG Chatbot service for response personalization.
- **FR-009**: System MUST NOT implement a custom auth system; it must rely entirely on Better-Auth primitives.
- FR-010**: System MUST strictly enforce profile completion by redirecting any authenticated user with an incomplete profile to the setup page if they attempt to access protected routes.
- **FR-011**: System MUST treat profile data as immutable once submitted; users cannot update their proficiency or role after the onboarding phase.
- **FR-012**: System MUST support Email/Password, Google, and GitHub authentication methods.

### Key Entities *(include if feature involves data)*

- **User**: The core identity entity managed by Better-Auth (contains email, password hash, verified status, etc.).
- **UserProfile**: Extension entity containing 'python_proficiency' and 'developer_role', linked 1:1 to the User entity.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of new signups successfully capture and persist both Python proficiency and Developer role.
- **SC-002**: Unauthenticated users are blocked from RAG Chatbot 100% of the time.
- **SC-003**: Public book pages load for unauthenticated users with 0% redirection to login.
- **SC-004**: User profile context is retrieved within 100ms for use in Chatbot sessions.