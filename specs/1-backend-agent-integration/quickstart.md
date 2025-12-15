# Quickstart: Backend Agent Integration

## Prerequisites

- Python 3.9+
- Existing RAG agent built with OpenAI Agents SDK
- Qdrant Cloud (read-only access)
- Neon Serverless Postgres (for logging)
- Existing .env file with required environment variables
- ChatKit UI already set up

## Setup

1. **Install dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Configure environment**:
   Ensure your `.env` file exists at the project root with required variables:
   ```env
   OPENAI_API_KEY=your_api_key_here
   GEMINI_API_KEY=your_gemini_key_here  # For AsyncOpenAI-compatible client
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   NEON_DATABASE_URL=your_neon_db_url
   # Add other required environment variables
   ```

3. **Start the backend**:
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

## API Endpoints

### POST /chat
Send a message to the agent and receive a complete response.

**Request**:
```json
{
  "message": {
    "content": "Hello, how are you?",
    "user_id": "user-123",
    "timestamp": "2025-12-13T10:00:00Z"
  }
}
```

**Response**:
```json
{
  "response": {
    "content": "I'm doing well, thank you for asking!",
    "message_id": "msg-456",
    "timestamp": "2025-12-13T10:00:01Z",
    "status": "success"
  }
}
```

### POST /chat-stream
Send a message to the agent and receive a streaming response.

**Request**:
```json
{
  "message": {
    "content": "Tell me a story",
    "user_id": "user-123",
    "timestamp": "2025-12-13T10:00:00Z"
  }
}
```

**Response** (Server-Sent Events):
```text
data: {"content": "Once upon a time", "status": "partial"}

data: {"content": "there was a", "status": "partial"}

data: {"content": "beautiful kingdom", "status": "complete"}
```

## Environment Variables

- `OPENAI_API_KEY`: API key for OpenAI services
- `GEMINI_API_KEY`: API key for Gemini via AsyncOpenAI-compatible client
- `QDRANT_URL`: URL for Qdrant Cloud vector store (read-only)
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `NEON_DATABASE_URL`: Connection string for Neon Serverless Postgres
- `AGENT_TIMEOUT`: Timeout for agent responses (default: 30 seconds)
- `LOG_LEVEL`: Logging level (default: INFO)

## Testing

Run the backend tests:
```bash
cd backend
pytest tests/
```

## Development

To run with auto-reload during development:
```bash
uvicorn main:app --reload --port 8000
```

## Deployment

1. The backend is designed for Vercel Python Serverless Functions
2. Ensure all dependencies are in requirements.txt
3. The backend must remain stateless for serverless compatibility
4. Interaction logging will be stored in Neon database

## Interaction Logging

All interactions between ChatKit UI and the RAG agent are logged to Neon Serverless Postgres for analytics, including:
- Request and response content
- Timestamps and response time
- Error information if applicable
- Session and user identifiers (if available)