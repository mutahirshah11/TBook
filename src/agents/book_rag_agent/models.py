from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field

class ChunkMetadata(BaseModel):
    """Metadata associated with a text chunk."""
    file_path: str = Field(..., description="Source file path")
    chapter: Optional[str] = Field(None, description="Chapter title")
    heading: Optional[str] = Field(None, description="Section heading")

class RetrievedChunk(BaseModel):
    """A single chunk retrieved from the vector store."""
    text: str = Field(..., description="The content of the chunk")
    score: float = Field(..., description="Similarity score")
    metadata: ChunkMetadata = Field(..., description="Metadata of the chunk")

class RetrievalContext(BaseModel):
    """Collection of retrieved chunks."""
    chunks: List[RetrievedChunk] = Field(default_factory=list)

class RelevanceOutput(BaseModel):
    """Output for the relevance guardrail."""
    is_relevant: bool = Field(..., description="True if the query is relevant to the book/domain")
    reasoning: str = Field(..., description="Reasoning for the relevance decision")
