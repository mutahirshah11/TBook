# Tasks: Neon Postgres Chat History & Query Logging

**Feature**: `015-neon-chat-history`
**Status**: Pending

## Phase 1: Setup
**Goal**: Initialize project environment, install dependencies, and configure database connection.

- [ ] T001 Initialize project environment and create `requirements.txt` with dependencies (`asyncpg`, `python-dotenv`).
- [ ] T002 Create `.env` (or `.env.example`) with `DATABASE_URL` placeholder.
- [ ] T003 Create `src/database` directory structure.
- [ ] T004 Implement `src/database/connection.py` to handle `asyncpg` connection pool creation and lifecycle management.

## Phase 2: Foundational (Schema)
**Goal**: Create the necessary database tables.

- [ ] T005 Implement `scripts/setup_db.py` to connect to the database using the pool from T004.
- [ ] T006 Extend `scripts/setup_db.py` to execute SQL DDL for creating `user_queries` table (schema from `data-model.md`).
- [ ] T007 Extend `scripts/setup_db.py` to execute SQL DDL for creating `chat_history` table (schema from `data-model.md`).
- [ ] T008 Run `scripts/setup_db.py` and manually verify table creation (or verify via script output).

## Phase 3: User Story 2 - Store User Queries
**Goal**: Implement functionality to save user queries.
**Priority**: P1

- [ ] T009 [US2] Implement `save_user_query` function in `src/database/ops.py` with proper logging and error handling.
- [ ] T010 [US2] Create `scripts/test_us2.py` (or part of a larger test script) to invoke `save_user_query` with test data.
- [ ] T011 [US2] Verify `save_user_query` works by running the test script and checking logs/database.

## Phase 4: User Story 3 - Store Chat History
**Goal**: Implement functionality to save full chat history.
**Priority**: P1

- [ ] T012 [US3] Implement `save_chat_history` function in `src/database/ops.py` ensuring JSONB handling for `used_chunks`.
- [ ] T013 [US3] Create `scripts/test_us3.py` to invoke `save_chat_history` with test data including JSON context.
- [ ] T014 [US3] Verify `save_chat_history` works by running the test script.

## Phase 5: User Story 4 - Retrieve Chat History
**Goal**: Implement functionality to retrieve chat history for a user.
**Priority**: P2

- [ ] T015 [US4] Implement `get_chat_history` function in `src/database/ops.py` returning a list of dictionaries.
- [ ] T016 [US4] Create `scripts/test_us4.py` to invoke `get_chat_history` and verify output format and order.
- [ ] T017 [US4] Verify `get_chat_history` retrieves previously saved data correctly.

## Phase 6: Integration & Polish
**Goal**: Consolidate testing and cleanup.

- [ ] T018 Create a unified `scripts/test_db_operations.py` combining all story tests for a full end-to-end verification flow (Connect -> Insert Query -> Insert Chat -> Retrieve Chat).
- [ ] T019 Review code for logging consistency and proper error handling (try/except blocks in `ops.py`).

## Dependencies

- **US2, US3, US4** all depend on **Setup** (T001-T004) and **Foundational** (T005-T008) tasks.
- **US4** (Retrieval) effectively depends on **US3** (Storage) to have meaningful data to retrieve, though it can be implemented independently with manual data insertion.

## Parallel Execution Opportunities

- **T009 (save_user_query)** and **T012 (save_chat_history)** are independent and can be implemented in parallel once the database and connection logic are set up.
- **T015 (get_chat_history)** can also be implemented in parallel with the write operations.

## Implementation Strategy

1.  **Setup**: Get the database connected and tables created.
2.  **Writes**: Implement the "save" functions first to populate the DB.
3.  **Reads**: Implement the "get" function to verify the data.
4.  **Verify**: Run the end-to-end test suite.