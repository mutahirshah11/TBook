# CLI Script Interfaces

## 1. extract_book_chunks.py

**Purpose**: Reads markdown files and outputs structured chunks.

**Usage**:
```bash
python extract_book_chunks.py --docs-dir <path_to_docs> --output <path_to_output_json>
```

**Arguments**:
- `--docs-dir`: (Optional) Path to docs directory. Defaults to `../../docs`.
- `--output`: (Optional) Path to save processed chunks. Defaults to `chunks.json`.

**Output Format (JSON)**:
```json
[
  {
    "chunk_id": "md5_hash_of_content",
    "content": "Text content...",
    "metadata": {
      "source": "docs/intro.md",
      "chapter": "Introduction",
      "heading": "Welcome"
    }
  }
]
```

## 2. generate_embeddings.py

**Purpose**: Reads chunks, calls Cohere API, and saves embeddings.

**Usage**:
```bash
python generate_embeddings.py --input <chunks_json> --output <embeddings_json>
```

**Environment Variables**:
- `COHERE_API_KEY`: Required.

**Arguments**:
- `--input`: Path to chunks JSON. Defaults to `chunks.json`.
- `--output`: Path to save embeddings. Defaults to `embeddings.json`.

**Output Format (JSON)**:
```json
[
  {
    "id": "md5_hash_of_content",
    "vector": [0.123, -0.456, ...],
    "payload": {
      "content": "Text content...",
      "metadata": { ... }
    }
  }
]
```

## 3. push_to_qdrant.py

**Purpose**: Uploads vector data to Qdrant Cloud.

**Usage**:
```bash
python push_to_qdrant.py --input <embeddings_json>
```

**Environment Variables**:
- `QDRANT_URL`: Required.
- `QDRANT_API_KEY`: Required.

**Arguments**:
- `--input`: Path to embeddings JSON. Defaults to `embeddings.json`.

**Behavior**:
- Checks if collection `book_vectors` exists; creates it if not (1024 dim, Cosine).
- Performs upsert operations in batches.
