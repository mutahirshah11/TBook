# Feature Specification: Book RAG Chatbot Agent

**Feature Branch**: `016-book-rag-agent`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Build the standalone RAG Agent using the OpenAI Agents SDK..."

## User Scenarios & Testing

### User Story 1 - Answer Questions using Book Context (Priority: P1)

As a user, I want the agent to answer my questions using only the content of the technical book, so that I can get accurate information without hallucinations or outside knowledge.

**Why this priority**: This is the core value proposition of a RAG system for a specific book.

**Independent Test**: Can be tested by running the agent script with a query about a specific topic in the book and verifying the answer cites or reflects the book content.

**Acceptance Scenarios**:
1. **Given** the agent is running and connected to the vector store, **When** I ask "What is the definition of physical AI?", **Then** the agent retrieves relevant chunks and provides an answer based strictly on those chunks.
2. **Given** a query with no relevant content in the book, **When** I ask "How do I bake a cake?", **Then** the agent replies that it can only answer questions related to the book.

### User Story 2 - User-Selected Context Override (Priority: P2)

As a user, I want to highlight specific text and ask a question about it, so that the agent focuses on that specific passage instead of searching the whole book.

**Why this priority**: Allows users to get clarification on specific paragraphs they are reading.

**Independent Test**: Pass specific text as "selected_text" to the agent along with a query and verify the answer uses that text and does not perform a vector search.

**Acceptance Scenarios**:
1. **Given** I provide selected text "Robots use sensors..." and ask "Explain this", **Then** the agent explains the selected text without querying the vector database.
2. **Given** I provide selected text, **When** I ask a question, **Then** the logs show 0 retrieved chunks from Qdrant.

### User Story 3 - Debugging and Transparency (Priority: P3)

As a developer/tester, I want to see what chunks were retrieved and their similarity scores, so that I can tune the retrieval process.

**Why this priority**: Essential for verifying the "brain" is working correctly during development (as requested: "Add Logging and Debugging Utilities").

**Independent Test**: Run a query and check the console/logs for chunk IDs and scores.

**Acceptance Scenarios**:
1. **Given** a query is processed, **When** the answer is generated, **Then** the output includes the list of chunk IDs used and their similarity scores.
2. **Given** a query is processed, **When** the answer is generated, **Then** the raw prompt sent to the LLM is logged/accessible.

### Edge Cases

- **Empty Query**: Agent should return a user-friendly message prompting for input.
- **Service Downtime**: If Qdrant or the LLM API is down, the agent should report a connection error with detailed information (e.g., error codes, network issues, API status).
- **Token Limit Exceeded**: If the retrieved chunks + query exceed the context window, the system should prioritize the full user query, then remove the oldest chunks until the context fits.

## Assumptions & Constraints

- **Constraint**: Must use OpenAI Agents SDK syntax/patterns.
- **Constraint**: Must use AsyncOpenAI client.
- **Constraint**: Must use Gemini API Key (likely via compatible endpoint).
- **Constraint**: No UI or backend routes in this iteration (CLI/Script usage only).
- **Assumption**: Qdrant collection is already populated or scripts exist to populate it (based on "Connect to Qdrant Cloud").
- **Assumption**: "Gemini_API_KEY" implies access to a model capable of following the system instructions.

## Clarifications

### Session 2025-12-09
- Q: How should the streamed response be returned to the caller? → A: The `ask()` method returns an async iterator yielding response chunks.
- Q: How should the agent handle an empty query? → A: The agent responds with a message indicating the query is empty.
- Q: How should debug information be logged? → A: Debug information should be logged in a structured JSON/Dictionary format.
- Q: What level of detail should the agent provide when Qdrant or the LLM API is unavailable? → A: Detailed error information (e.g., error codes, network issues, API status).
- Q: If the combined context (retrieved chunks + user query) exceeds the LLM's token limit, how should the system prioritize or reduce the context? → A: Prioritize the full user query, then remove the oldest chunks.

## Requirements

### Functional Requirements

- **FR-001**: The system MUST accept a text query from the user.
- **FR-002**: The system MUST support an optional "selected text" input.
- **FR-003**: If "selected text" is provided, the system MUST use it as the sole context and bypass vector retrieval.
- **FR-004**: If "selected text" is NOT provided, the system MUST retrieve the top-N (default N=3) most relevant text chunks from the Qdrant vector store.
- **FR-005**: The system MUST construct a prompt that includes:
    - A system instruction to act as a technical book assistant.
    - A strict constraint to use ONLY provided context.
    - The retrieved or selected context (formatted with metadata like chapter, section).
    - The user's query.
- **FR-006**: The system MUST generate a response using an LLM (asynchronously).
- **FR-007**: The system MUST stream the response back to the caller by returning an asynchronous generator that yields response chunks.
- **FR-008**: The system MUST refuse to answer questions unrelated to the provided context (Guardrails).
- **FR-009**: The system MUST log debug information for each request in a structured JSON/Dictionary format, including:
    - Retrieved chunk IDs.
    - Similarity scores.
    - Raw context passed to the LLM.