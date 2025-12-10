# Interface Contract: Book RAG Agent

**Status**: Phase 1 Design
**Type**: Python API (CLI/Library)

## CLI Interface

The agent is exposed via a CLI script: `scripts/rag_agent.py`.

### Arguments

| Argument | Type | Required | Description |
|----------|------|----------|-------------|
| `--query` | string | No | The user's question. |
| `--selected-text` | string | No | User-selected context override. |
| `--interactive` | flag | No | specific interactive mode loop. |

### Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| `GEMINI_API_KEY` | Yes | API Key for Gemini (LLM). |
| `QDRANT_URL` | Yes | URL for Qdrant Vector DB. |
| `QDRANT_API_KEY` | Yes | API Key for Qdrant. |
| `COHERE_API_KEY` | Yes | API Key for Cohere (Embeddings). |

## Python Interface

```python
# src/agents/book_rag_agent/interface.py

async def ask_agent(query: str, selected_text: Optional[str] = None) -> AsyncGenerator[str, None]:
    """
    Invokes the BookRAGAgent.
    
    Args:
        query: The user's question.
        selected_text: Optional text override.
        
    Yields:
        String chunks of the response.
    """
```
