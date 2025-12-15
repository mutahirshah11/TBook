---
id: 001
title: Spec for Qdrant Book Vectors
stage: spec
date: 2025-12-08
surface: agent
model: gemini-2.0-flash-exp
feature: 014-qdrant-book-vectors
branch: 014-qdrant-book-vectors
user: Mutahir
command: /sp.specify
labels: ["spec", "qdrant", "rag"]
links:
  spec: specs/014-qdrant-book-vectors/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/014-qdrant-book-vectors/spec.md
  - specs/014-qdrant-book-vectors/checklists/requirements.md
tests: []
---

## Prompt

# Task: Iteration 1 – Qdrant Vector Database & Book Embeddings You are required to generate all the specifications, Python scripts, and setup for Iteration 1 of a RAG chatbot project. The goal of this iteration is to extract book content, split it into chunks, generate embeddings, and store them in Qdrant Cloud. This iteration will serve as the foundation for the entire RAG system. --- ## Requirements 1. **Qdrant Setup** - Connect to Qdrant Cloud (free-tier). - Create a collection named `book_vectors`. - Set vector size compatible with embedding model. - Ensure each vector stores metadata: `chapter`, `heading`, `chunk_id`, `file_path`. 2. **Book Content Extraction** - Read all markdown files from the Docusaurus `docs/` folder. - Extract clean text from markdown. - Split text into chunks of 500–1000 tokens. - Keep metadata for each chunk for retrieval. 3. **Embedding Generation** - Use the embedding model: `cohere` (as specified in project settings). - Each chunk must produce an embedding vector. - Handle API rate limits and errors. - Store embedding vectors along with metadata. 4. **Push to Qdrant** - Use Python `qdrant-client`. - Scripts must include: - `create_collection()` - `upload_vectors(vectors_with_metadata)` - Ensure that all vectors are stored correctly. 5. **Scripts** - `extract_book_chunks.py` – extracts and splits text with metadata. - `generate_embeddings.py` – generates embeddings using cohere. - `push_to_qdrant.py` – uploads embeddings to Qdrant. - Include clear comments in code. - Make scripts modular for reuse in future iterations. 6. **Deliverables** - Python scripts listed above. - Fully populated Qdrant collection `book_vectors`. - Metadata stored with each chunk for retrieval.

## Response snapshot

Created spec for Qdrant setup and book embedding extraction.

## Outcome

- ✅ Impact: Feature spec created and validated.
- 🧪 Tests: N/A
- 📁 Files: Created spec.md and requirements checklist.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Successfully mapped user requirements to structured spec.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): N/A
