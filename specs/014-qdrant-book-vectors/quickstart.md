# Quickstart: Qdrant Vector Setup

## Prerequisites

1. **Python Environment**:
   ```bash
   pip install -r requirements.txt
   ```
   (See `requirements.txt` for dependencies: `qdrant-client`, `cohere`, `langchain-text-splitters`, `python-dotenv`)

2. **Environment Variables**:
   Create a `.env` file in the root or scripts directory:
   ```env
   COHERE_API_KEY=your_cohere_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_key
   ```

## Workflow

### 1. Extract Content
Parse the documentation and create chunks.
```bash
python scripts/extract_book_chunks.py
```
*Outputs: `chunks.json`*

### 2. Generate Embeddings
Convert chunks into vectors using Cohere.
```bash
python scripts/generate_embeddings.py
```
*Outputs: `embeddings.json`*

### 3. Upload to Qdrant
Push vectors and metadata to the cloud.
```bash
python scripts/push_to_qdrant.py
```
*Verifies collection and upserts data.*

## Verification

Check the collection info:
```python
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
print(client.get_collection("book_vectors"))
```
