# Book RAG Agent

A structured RAG agent for the Robotics Book, built with the OpenAI Agents SDK and powered by Gemini.

## Directory Structure

- `agent.py`: Defines the main `BookRAGAgent` and its instructions.
- `tools.py`: Implements the `retrieve_book_context` tool logic.
- `guardrails.py`: Implements the `relevance_guardrail` using a lightweight agent.
- `interface.py`: Provides the `ask_agent` async generator interface.
- `models.py`: Pydantic models for data structures.
- `settings.py`: Configuration and client initialization (Qdrant, Cohere, OpenAI/Gemini).
- `utils.py`: Shared utilities (logging).

## Usage

See `scripts/rag_agent.py` for the CLI implementation.

```bash
python scripts/rag_agent.py --interactive
```

## Configuration

Ensure `.env` contains:
- `GEMINI_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `COHERE_API_KEY`
