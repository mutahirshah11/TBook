# Data Model: Book RAG Chatbot Agent

**Status**: Phase 1 Design
**Spec**: [spec.md](spec.md)

## Agents

### `BookRAGAgent`
The primary agent responsible for answering user questions using book context.

| Property | Type | Description |
|----------|------|-------------|
| `name` | string | "BookRAGAgent" |
| `instructions` | string | System prompt defining RAG behavior (prioritize context, refuse outside knowledge). |
| `model` | string | "gemini-1.5-flash" (via OpenAI Compat) |
| `tools` | list | `[retrieve_book_context]` |
| `input_guardrails` | list | `[relevance_guardrail]` |

### `GuardrailAgent`
A lightweight agent used to validate input relevance.

| Property | Type | Description |
|----------|------|-------------|
| `name` | string | "RelevanceGuardrail" |
| `instructions` | string | "Check if the user's query is related to robotics, AI, or the book's domain." |
| `output_type` | Pydantic | `RelevanceOutput` |

## Data Structures

### `RelevanceOutput` (Pydantic)
Structured output for the guardrail.

```python
class RelevanceOutput(BaseModel):
    is_relevant: bool
    reasoning: str
```

### `RetrievalContext` (Pydantic - Internal)
Used within the tool to structure the returned context.

```python
class ChunkMetadata(BaseModel):
    file_path: str
    chapter: str
    heading: str

class RetrievedChunk(BaseModel):
    text: str
    score: float
    metadata: ChunkMetadata
```

## Storage Schema (Qdrant)

*Existing Collection: `book_vectors`*

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID/Hash | Content hash |
| `vector` | float[] | 1024d (Cohere) |
| `payload.content` | string | Text chunk |
| `payload.file_path` | string | Source file |
| `payload.chapter` | string | Chapter name |
