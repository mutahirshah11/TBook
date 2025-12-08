from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional

@dataclass
class Chunk:
    """
    Represents a processed segment of text from a source document.
    """
    id: str  # Unique identifier for the chunk, typically a hash of its content and metadata
    content: str
    file_path: str  # Path to the source markdown file
    heading: str  # Nearest preceding header (H1-H3)
    chapter: str  # Chapter name, derived from file hierarchy
    token_count: int # Number of tokens in this chunk

@dataclass
class VectorRecord:
    """
    Represents a vector with its payload, ready for storage in Qdrant.
    """
    id: str  # The unique ID for the vector in Qdrant, same as Chunk.id
    vector: List[float] # The embedding vector
    payload: Dict[str, Any] # Metadata associated with the vector (e.g., from Chunk)

# Example of how payload would be structured based on data-model.md:
# payload = {
#     "chunk_id": "...",
#     "content": "...",
#     "file_path": "...",
#     "heading": "...",
#     "chapter": "..."
# }
