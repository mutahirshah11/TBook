# Tasks: Qdrant Vector Database & Book Embeddings (Iteration 1)

**Feature**: `014-qdrant-book-vectors`
**Status**: Pending

## Phase 1: Setup
**Goal**: Initialize project environment and install dependencies.

- [x] T001 Initialize Python environment and create `requirements.txt` with dependencies (`qdrant-client`, `cohere`, `langchain-text-splitters`, `python-dotenv`).
- [x] T002 Create `.env.example` file with placeholders for `COHERE_API_KEY`, `QDRANT_URL`, and `QDRANT_API_KEY`.
- [x] T003 Create `scripts/` directory for hosting the python scripts.

## Phase 2: Foundational
**Goal**: Create reusable utilities and data models.

- [x] T004 Implement `Chunk` and `VectorRecord` data classes/models in `scripts/models.py` (or shared util) matching `data-model.md`.
- [x] T005 Implement `calculate_content_hash` utility in `scripts/utils.py` for idempotent ID generation (MD5/SHA256).

## Phase 3: User Story 1 - Qdrant Environment Setup
**Goal**: Establish connection to Qdrant Cloud and initialize collection.
**Priority**: P1

- [x] T006 [US1] Implement `create_collection_if_not_exists` function in `scripts/qdrant_utils.py` with 1024 dim and Cosine metric.
- [x] T007 [US1] Create `scripts/setup_qdrant.py` (or part of push script) to run the collection creation logic independently for verification.
- [x] T008 [US1] Verify Qdrant connection and collection creation (Manual verification via dashboard or script output).

## Phase 4: User Story 2 - Content Extraction and Chunking
**Goal**: Read docs and split into chunks.
**Priority**: P1

- [x] T009 [US2] Implement `extract_text_from_markdown` function in `scripts/extract_book_chunks.py` to recursively read `.md` files from `docs/`.
- [x] T010 [US2] Implement chunking logic using `TokenTextSplitter` (500-1000 tokens, 20% overlap) in `scripts/extract_book_chunks.py`.
- [x] T011 [US2] Implement metadata extraction (chapter, heading, file_path) for each chunk.
- [x] T012 [US2] Implement JSON export in `scripts/extract_book_chunks.py` matching `contracts/scripts-api.md` output format.
- [x] T013 [US2] Verify extraction script runs on `docs/` and produces valid `chunks.json`.

## Phase 5: User Story 3 - Embedding Generation and Storage
**Goal**: Generate embeddings and upload to Qdrant.
**Priority**: P1

- [x] T014 [US3] Implement `scripts/generate_embeddings.py` to read `chunks.json`.
- [x] T015 [US3] Integrate `cohere.Client` to generate embeddings for chunks (handle rate limits with simple backoff/sleep).
- [x] T016 [US3] Save intermediate `embeddings.json` with vectors and payload.
- [x] T017 [US3] Implement `scripts/push_to_qdrant.py` to read `embeddings.json`.
- [x] T018 [US3] Implement batch upsert logic using `qdrant_client` and the deterministic IDs from T005.
- [x] T019 [US3] Verify end-to-end flow: Extract -> Embed -> Push -> Query count.

## Dependencies

- **US2 (Extraction)** and **US3 (Embedding/Storage)** depend on **Setup** and **Foundational** tasks.
- **US3** depends on valid output from **US2** (chunks).
- **US1** (Setup Qdrant) is technically independent but required before **US3** (Push) can succeed.

## Parallel Execution Opportunities

- **T006 (Qdrant Setup)** and **T009 (Extraction Logic)** can be developed in parallel.
- **T014 (Embedding Logic)** can be developed in parallel with **T017 (Push Logic)** if mock data is used.

## Implementation Strategy

1. **Setup**: Get the environment ready.
2. **US1 & US2**: Parallel track - set up DB and start extracting local text.
3. **US3**: Connect the pipes - generate vectors and push to the DB setup in US1.
4. **Verify**: Run the full sequence as described in `quickstart.md`.
