import logging
from typing import List

from src.agents.book_rag_agent.settings import settings
from src.agents.book_rag_agent.models import RetrievedChunk, ChunkMetadata

logger = logging.getLogger(__name__)

from agents import function_tool

@function_tool
def retrieve_book_context(query: str) -> str:
    """
    Retrieves relevant book chunks from the vector database.
    Use this tool when the user asks a question about the book's content.
    """
    return retrieve_book_context_logic(query)

def retrieve_book_context_logic(query: str, limit: int = 5) -> str:
    """
    Core logic for retrieving book context. 
    Separated from the tool decorator for easier testing.
    """
    logger.info(f"Retrieving context for query: {query}")
    
    qdrant = settings.get_qdrant_client()
    cohere_client = settings.get_cohere_client()
    
    if not qdrant or not cohere_client:
        msg = "Vector DB or Embedding client not available."
        logger.error(msg)
        return f"Error: {msg}"

    try:
        # Embed query
        embed_resp = cohere_client.embed(
            texts=[query],
            model=settings.EMBEDDING_MODEL,
            input_type="search_query"
        )
        query_vector = embed_resp.embeddings[0]
        
        # Search Qdrant
        search_result = qdrant.query_points(
            collection_name=settings.COLLECTION_NAME,
            query=query_vector,
            limit=limit
        )
        
        chunks: List[RetrievedChunk] = []
        for point in search_result.points:
            payload = point.payload or {}
            
            metadata = ChunkMetadata(
                file_path=str(payload.get("file_path", "unknown")),
                chapter=str(payload.get("chapter", "")),
                heading=str(payload.get("heading", ""))
            )
            
            chunk = RetrievedChunk(
                text=str(payload.get("content", "")),
                score=point.score,
                metadata=metadata
            )
            chunks.append(chunk)
            
        # Format for LLM
        formatted_chunks = []
        for i, c in enumerate(chunks):
            source_info = f"{c.metadata.file_path}"
            if c.metadata.chapter:
                source_info += f" | {c.metadata.chapter}"
            
            formatted_chunks.append(
                f"--- Chunk {i+1} (Score: {c.score:.2f}) ---\n"
                f"Source: {source_info}\n"
                f"Content: {c.text}\n"
            )
            
        result_str = "\n".join(formatted_chunks)
        if not result_str:
            return "No relevant book context found."
            
        return result_str

    except Exception as e:
        logger.error(f"Retrieval failed: {e}", exc_info=True)
        return f"Error during retrieval: {str(e)}"

    # Structured Logging
    log_payload = {
        "event": "retrieval",
        "query": query,
        "count": len(chunks),
        "top_score": chunks[0].score if chunks else 0.0,
        "chunks": [{"id": c.metadata.file_path, "score": c.score} for c in chunks]
    }
    logger.info(f"Retrieval Debug: {log_payload}")
    
    return result_str
