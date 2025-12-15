# Backend Agent Integration

This backend service connects the ChatKit UI to the existing RAG agent, providing a bridge between the frontend interface and the AI agent functionality. The service is built with FastAPI and designed for deployment as Python serverless functions on Vercel.

## Features

- **Synchronous Chat API**: Send messages to the agent and receive complete responses
- **Streaming Chat API**: Real-time streaming of agent responses using Server-Sent Events
- **Interaction Logging**: All interactions are logged to Neon Serverless Postgres for analytics
- **Error Handling**: Comprehensive error handling and validation
- **Timeout Management**: Configurable request timeouts to prevent hanging requests

## API Endpoints

### POST /api/v1/chat
Send a message to the agent and receive a complete response.

**Request Body**:
```json
{
  "message": {
    "content": "Your message here",
    "user_id": "optional-user-id",
    "metadata": {
      "context": "full-book"
    }
  }
}
```

**Response**:
```json
{
  "response": {
    "content": "Agent's response",
    "message_id": "user-id-or-unknown",
    "timestamp": "2025-12-13T10:00:00Z",
    "status": "success"
  }
}
```

### POST /api/v1/chat/stream
Send a message to the agent and receive a streaming response.

**Request Body**:
```json
{
  "message": {
    "content": "Your message here",
    "user_id": "optional-user-id",
    "metadata": {
      "context": "full-book"
    }
  }
}
```

**Response**: Server-Sent Events stream with JSON payloads:
```
data: {"content": "Partial response", "status": "partial"}

data: {"content": "Final response", "status": "success"}
```

## Environment Variables

Create a `.env` file in the project root with the following variables:

```env
OPENAI_API_KEY=your_api_key_here
GEMINI_API_KEY=your_gemini_key_here
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_db_url
AGENT_TIMEOUT=30
LOG_LEVEL=INFO
```

## Setup and Installation

1. **Install dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Configure environment**:
   Ensure your `.env` file exists at the project root with required variables

3. **Start the backend**:
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

## Testing

Run the backend tests:
```bash
cd backend
pytest tests/
```

## Deployment

1. The backend is designed for Vercel Python Serverless Functions
2. Ensure all dependencies are in requirements.txt
3. The backend must remain stateless for serverless compatibility
4. Interaction logging will be stored in Neon database

## Architecture

- **Framework**: FastAPI for async operations and streaming support
- **Validation**: Pydantic for request/response validation
- **Database**: Neon Serverless Postgres for interaction logging
- **Vector Store**: Qdrant Cloud (read-only access)
- **Deployment**: Vercil Python Serverless Functions

## Security

- Input validation using Pydantic schemas
- CORS configured for secure cross-origin requests
- Environment variables for sensitive configuration
- Error messages sanitized to prevent information disclosure