# Feature Specification: Qdrant Vector Database & Book Embeddings (Iteration 1)

**Feature Branch**: `014-qdrant-book-vectors`
**Created**: 2025-12-08
**Status**: Draft
**Input**: User description: "# Task: Iteration 1 – Qdrant Vector Database & Book Embeddings You are required to generate all the specifications, Python scripts, and setup for Iteration 1 of a RAG chatbot project. The goal of this iteration is to extract book content, split it into chunks, generate embeddings, and store them in Qdrant Cloud. This iteration will serve as the foundation for the entire RAG system. --- ## Requirements 1. **Qdrant Setup** - Connect to Qdrant Cloud (free-tier). - Create a collection named `book_vectors`. - Set vector size compatible with embedding model. - Ensure each vector stores metadata: `chapter`, `heading`, `chunk_id`, `file_path`. 2. **Book Content Extraction** - Read all markdown files from the Docusaurus `docs/` folder. - Extract clean text from markdown. - Split text into chunks of 500–1000 tokens. - Keep metadata for each chunk for retrieval. 3. **Embedding Generation** - Use the embedding model: `cohere` (as specified in project settings). - Each chunk must produce an embedding vector. - Handle API rate limits and errors. - Store embedding vectors along with metadata. 4. **Push to Qdrant** - Use Python `qdrant-client`. - Scripts must include: - `create_collection()` - `upload_vectors(vectors_with_metadata)` - Ensure that all vectors are stored correctly. 5. **Scripts** - `extract_book_chunks.py` – extracts and splits text with metadata. - `generate_embeddings.py` – generates embeddings using cohere. - `push_to_qdrant.py` – uploads embeddings to Qdrant. - Include clear comments in code. - Make scripts modular for reuse in future iterations. 6. **Deliverables** - Python scripts listed above. - Fully populated Qdrant collection `book_vectors`. - Metadata stored with each chunk for retrieval."

## Clarifications

### Session 2025-12-08

- Q: Cohere Embedding Model → A: embed-english-v3.0 (1024 dimensions)
- Q: Distance Metric → A: Cosine Similarity
- Q: Chunking Strategy → A: TokenTextSplitter
- Q: Idempotency Strategy for Uploads → A: Upsert with content hash as ID
- Q: Handling Chunk Overlap → A: 20% of chunk size

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Qdrant Environment Setup (Priority: P1)

As a developer, I want to establish a connection to Qdrant Cloud and initialize the vector collection so that the system is ready to store embeddings.

**Why this priority**: Foundation for storage; nothing else can be stored without this.

**Independent Test**: Can be tested by running the collection creation script and verifying the collection exists in Qdrant Cloud dashboard or via API info check.

**Acceptance Scenarios**:

1. **Given** valid Qdrant Cloud credentials, **When** the setup script runs, **Then** a collection named `book_vectors` is created.
2. **Given** the collection is created, **When** inspecting configuration, **Then** the vector size is `1024` dimensions (for Cohere `embed-english-v3.0`) and distance metric is `Cosine`.
3. **Given** invalid credentials, **When** the script runs, **Then** an appropriate error message is displayed.

### User Story 2 - Content Extraction and Chunking (Priority: P1)

As a system, I need to read documentation files and split them into manageable chunks so that they can be processed by the embedding model.

**Why this priority**: Raw data must be prepared before embedding.

**Independent Test**: Run extraction script on a sample docs folder and verify output structure (text chunks + metadata) without needing Qdrant or Embeddings.

**Acceptance Scenarios**:

1. **Given** a directory of markdown files, **When** the extraction script runs, **Then** it produces a structured list of text chunks using `TokenTextSplitter`.
2. **Given** a markdown file, **When** processed, **Then** it is split into chunks between 500-1000 tokens with a 20% overlap.
3. **Given** a markdown file, **When** processed, **Then** each chunk contains metadata: `chapter`, `heading`, `chunk_id`, `file_path`.
4. **Given** markdown syntax (headers, links), **When** extracted, **Then** clean text is preserved/cleaned appropriately.

### User Story 3 - Embedding Generation and Storage (Priority: P1)

As a system, I need to generate vector embeddings for each text chunk and upload them to Qdrant so that the RAG system can perform semantic searches.

**Why this priority**: Core value of the iteration; populates the database.

**Independent Test**: Mock the embedding API to return dummy vectors, verify upload logic works. Or run on small subset to verify end-to-end flow.

**Acceptance Scenarios**:

1. **Given** a list of text chunks, **When** the embedding script runs, **Then** it calls the Cohere API (`embed-english-v3.0`) and receives vector embeddings.
2. **Given** API rate limits, **When** limits are hit, **Then** the script handles them gracefully (retries or waits) without crashing.
3. **Given** generated embeddings, **When** upload script runs, **Then** vectors are upserted into `book_vectors` collection with their metadata, using a content hash as the ID.
4. **Given** the upload is complete, **When** querying the collection count, **Then** it matches the number of unique processed chunks.

### Edge Cases

- What happens when a file is empty or contains only images?
- What happens if Qdrant Cloud is down?
- What happens if a chunk exceeds the token limit of the embedding model?
- What happens if the API key is invalid?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST connect to Qdrant Cloud using provided credentials.
- **FR-002**: System MUST create a collection `book_vectors` if it does not exist.
- **FR-003**: System MUST configure the collection vector size to `1024` dimensions (Cohere `embed-english-v3.0`) and use `Cosine` distance metric.
- **FR-004**: System MUST recursively read all `.md` files from the `docs/` directory.
- **FR-005**: System MUST extract plain text from markdown, removing or handling formatting syntax where appropriate.
- **FR-006**: System MUST split text into chunks of 500-1000 tokens with a 20% overlap, using `TokenTextSplitter`.
- **FR-007**: Each chunk MUST include metadata: `chapter` (derived from hierarchy/filename), `heading` (nearest header), `chunk_id` (unique), and `file_path`.
- **FR-008**: System MUST use Cohere API (`embed-english-v3.0`) to generate embeddings for each chunk.
- **FR-009**: System MUST handle API rate limiting (e.g., HTTP 429) by waiting/retrying.
- **FR-010**: System MUST upsert vectors and metadata to the Qdrant collection, using a content hash as the vector ID to ensure idempotency.
- **FR-011**: Scripts MUST be modular: `extract_book_chunks.py`, `generate_embeddings.py`, `push_to_qdrant.py`.

### Key Entities

- **Chunk**: A segment of text from the documentation, containing `content`, `metadata` (chapter, heading, id, path).
- **Embedding**: A vector representation of a Chunk (float array).
- **Collection**: The Qdrant storage unit containing Vectors and Payload (metadata).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Qdrant collection `book_vectors` exists and contains vectors for 100% of valid markdown files in `docs/`.
- **SC-002**: Each stored vector has non-null metadata fields: `chapter`, `heading`, `chunk_id`, `file_path`.
- **SC-003**: Extraction and Embedding process completes for the entire book without unhandled exceptions.
- **SC-004**: Vector search (manual test) returns relevant chunks for a sample query.