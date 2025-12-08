# Research: Qdrant Vector Database & Book Embeddings

**Feature**: `014-qdrant-book-vectors`
**Status**: Complete
**Date**: 2025-12-08

## 1. Technical Decisions

### Embedding Model
- **Decision**: `cohere.embed-english-v3.0`
- **Rationale**: High performance on English text, optimized for RAG tasks (clustering/retrieval), and fits within the free/low-cost tier usage for development.
- **Dimensions**: 1024

### Vector Database
- **Decision**: Qdrant Cloud (Free Tier)
- **Rationale**: Easy setup, managed service, good Python client support, and sufficient limits for a single book's worth of vectors.
- **Distance Metric**: Cosine Similarity (Standard for semantic search with normalized embeddings).

### Chunking Strategy
- **Decision**: `TokenTextSplitter` from LangChain
- **Rationale**: Ensures chunks fit within the model's context window (tokens) more reliably than character splitting.
- **Parameters**:
    - Chunk Size: 500-1000 tokens (target ~800)
    - Overlap: 20% (preserves context at boundaries)

### Idempotency
- **Decision**: Deterministic IDs based on content hash.
- **Rationale**: Allows re-running the ingestion pipeline without creating duplicate vectors. If content changes, hash changes -> new vector (old one remains unless explicitly cleaned, but for this iteration, simple upsert is sufficient).
- **Implementation**: MD5 or SHA256 of `(file_path + chunk_index + content)`.

## 2. Library Selection

- **Qdrant**: `qdrant-client`
- **Embeddings**: `cohere`
- **Chunking**: `langchain-text-splitters`

## 3. Unknowns & Resolutions

- *All critical unknowns were resolved during the specification clarification phase.*
