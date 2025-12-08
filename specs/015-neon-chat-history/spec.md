# Feature Specification: Neon Postgres Chat History & Query Logging

**Feature Branch**: `015-neon-chat-history`
**Created**: 2025-12-08
**Status**: Draft
**Input**: User description: "Goal: Set up Neon Postgres to store user queries and chat history, and connect it to FastAPI for logging. Requirements / Tasks: Create tables: user_queries(id, user_id, query, timestamp) chat_history(id, user_id, query, response, used_chunks JSONB, timestamp) Provide helper functions for database operations: save_user_query(user_id, query) save_chat_history(user_id, query, response, used_chunks) get_chat_history(user_id) Ensure error handling and logging for all database operations. Note: No backend or API integration yet; this is purely the database and utility functions. Deliverables: Tables created and ready for future use Helper functions for database operations Tested database connectivity Fully prepared for agent and backend integration in the next iterations"

## Clarifications

### Session 2025-12-08

- Q: Database Driver → A: asyncpg
- Q: User ID Type → A: UUID
- Q: Connection Pooling → A: asyncpg connection pool
- Q: Environment Variable → A: DATABASE_URL
- Q: Logging Library → A: Standard Python `logging` module

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Database Setup & Connection (Priority: P1)

As a developer, I want to connect to a Neon Postgres database and ensure the necessary tables are created so that chat interactions and user queries can be persisted.

**Why this priority**: This is the foundational step; no data can be stored without the database and tables.

**Independent Test**: Can be tested by running a setup script and verifying table existence and schema in the database.

**Acceptance Scenarios**:

1.  **Given** valid Neon Postgres connection details in `DATABASE_URL`, **When** the database setup script runs, **Then** a connection is established successfully using the `asyncpg` driver with connection pooling.
2.  **Given** a connected database, **When** the setup script runs, **Then** `user_queries` table exists with columns `id` (UUID), `user_id` (UUID), `query` (TEXT), `timestamp`.
3.  **Given** a connected database, **When** the setup script runs, **Then** `chat_history` table exists with columns `id` (UUID), `user_id` (UUID), `query` (TEXT), `response` (TEXT), `used_chunks` (JSONB), `timestamp`.
4.  **Given** invalid connection details, **When** the setup script attempts to connect, **Then** an appropriate error is logged using the standard `logging` module, and the connection fails gracefully.

### User Story 2 - Store User Queries (Priority: P1)

As a system, I want to save individual user queries to the database so that a log of all interactions is maintained.

**Why this priority**: Essential for basic logging and auditing of user input.

**Independent Test**: Call the `save_user_query` function, then query the database directly to confirm the entry.

**Acceptance Scenarios**:

1.  **Given** a valid `user_id` (UUID) and `query` string, **When** `save_user_query` is called, **Then** a new record is inserted into the `user_queries` table.
2.  **Given** the record is inserted, **When** retrieving from `user_queries`, **Then** `id` (UUID), `user_id` (UUID), `query`, and `timestamp` fields match the input and creation time.
3.  **Given** a database error occurs during saving, **When** `save_user_query` is called, **Then** the error is logged using standard `logging`, and no record is inserted.

### User Story 3 - Store Chat History (Priority: P1)

As a system, I want to save complete chat turns (query, response, used chunks) to the database so that full conversation history is preserved.

**Why this priority**: Critical for maintaining context and understanding agent performance.

**Independent Test**: Call the `save_chat_history` function, then query the database directly to confirm the entry, including the JSONB field.

**Acceptance Scenarios**:

1.  **Given** valid `user_id` (UUID), `query`, `response`, and `used_chunks` (JSONB), **When** `save_chat_history` is called, **Then** a new record is inserted into the `chat_history` table.
2.  **Given** the record is inserted, **When** retrieving from `chat_history`, **Then** `id` (UUID), `user_id` (UUID), `query`, `response`, `used_chunks`, and `timestamp` fields match the input and creation time.
3.  **Given** a database error occurs during saving, **When** `save_chat_history` is called, **Then** the error is logged using standard `logging`, and no record is inserted.

### User Story 4 - Retrieve Chat History (Priority: P2)

As a system, I want to retrieve all chat history for a specific user so that previous interactions can be reloaded or analyzed.

**Why this priority**: Necessary for providing context to the agent or for user-facing history features.

**Independent Test**: Insert multiple chat history entries for a user, then call `get_chat_history` and verify all entries are returned correctly.

**Acceptance Scenarios**:

1.  **Given** a `user_id` (UUID) with existing chat history, **When** `get_chat_history` is called, **Then** a list of `chat_history` records for that user is returned.
2.  **Given** a `user_id` (UUID) with no existing chat history, **When** `get_chat_history` is called, **Then** an empty list is returned.
3.  **Given** a database error occurs during retrieval, **When** `get_chat_history` is called, **Then** the error is logged using standard `logging`, and an appropriate empty/error response is returned.

### Edge Cases

- What happens if `used_chunks` JSONB data is malformed? -> Should be validated or handled gracefully during saving.
- What are the performance implications of `get_chat_history` for users with very long histories? -> Initial implementation can retrieve all, but a future iteration might require pagination.
- What happens if `user_id` does not exist in any other system? (This feature only handles persistence, not user management).
- What happens if the database connection is lost during an operation? -> Appropriate error handling and retry mechanism.

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: System MUST connect to a Neon Postgres database using the `asyncpg` driver with connection pooling and credentials from `DATABASE_URL`.
-   **FR-002**: System MUST create `user_queries` table with schema `(id UUID PRIMARY KEY, user_id UUID, query TEXT, timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP)`.
-   **FR-003**: System MUST create `chat_history` table with schema `(id UUID PRIMARY KEY, user_id UUID, query TEXT, response TEXT, used_chunks JSONB, timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP)`.
-   **FR-004**: System MUST provide a `save_user_query(user_id: UUID, query: str)` helper function to insert records into `user_queries`.
-   **FR-005**: System MUST provide a `save_chat_history(user_id: UUID, query: str, response: str, used_chunks: Dict)` helper function to insert records into `chat_history`.
-   **FR-006**: System MUST provide a `get_chat_history(user_id: UUID)` helper function to retrieve all records from `chat_history` for a given user, ordered by timestamp.
-   **FR-007**: All database operations MUST include robust error handling, logging failures via the standard Python `logging` module.
-   **FR-008**: All database operations MUST log successful execution (e.g., "Query saved successfully") via the standard Python `logging` module.

### Key Entities

-   **user_queries**: Table storing individual user input queries.
-   **chat_history**: Table storing complete conversational turns, including agent responses and relevant context.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: Database setup script executes without errors, creating both `user_queries` and `chat_history` tables.
-   **SC-002**: `save_user_query` function successfully inserts a query into `user_queries` table within 500ms.
-   **SC-003**: `save_chat_history` function successfully inserts a chat entry (including JSONB `used_chunks`) into `chat_history` table within 500ms.
-   **SC-004**: `get_chat_history` function retrieves all relevant records for a user with 100% accuracy within 1 second for up to 100 entries.
-   **SC-005**: All database helper functions log both success and error events using the standard `logging` library.