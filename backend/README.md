# Backend Agent Integration

This backend service connects the ChatKit UI to the existing RAG agent, providing a bridge between the frontend interface and the AI agent functionality. The service is built with FastAPI and includes user authentication and conversation history storage with FastAPI lifespan management. The service is designed for deployment as Python serverless functions on Vercel.

## Features

- **Synchronous Chat API**: Send messages to the agent and receive complete responses
- **Streaming Chat API**: Real-time streaming of agent responses using Server-Sent Events
- **User Authentication**: Foundation for user accounts with email and password storage (preparing for Better Auth integration)
- **Conversation History**: User-specific conversation history with automatic retention of last 50 conversations
- **Interaction Logging**: All interactions are logged to Neon Serverless Postgres for analytics
- **Database Connection Management**: Proper initialization and cleanup using FastAPI lifespan event handlers
- **Error Handling**: Comprehensive error handling and validation with graceful degradation when database is unavailable
- **Timeout Management**: Configurable request timeouts to prevent hanging requests

## API Endpoints

### Authentication Endpoints

#### POST /api/v1/auth/register
Register a new user account.

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "securepassword123"
}
```

**Response**:
```json
{
  "id": "user-uuid",
  "email": "user@example.com",
  "created_at": "2023-12-21T10:00:00Z"
}
```

#### POST /api/v1/auth/login
Authenticate a user account.

**Request Body**:
```json
{
  "email": "user@example.com",
  "password": "securepassword123"
}
```

**Response**:
```json
{
  "user_id": "user-uuid",
  "email": "user@example.com"
}
```

### Conversation Endpoints

#### POST /api/v1/conversations/internal-save
Internal endpoint to save a conversation (used by chat functionality).

**Request Body**:
```json
{
  "user_id": "user-uuid",
  "query": "User's query",
  "response": "AI's response"
}
```

### Chat Endpoints

#### POST /api/v1/chat
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
DATABASE_URL=postgresql://username:password@host:port/database
AGENT_TIMEOUT=30
LOG_LEVEL=INFO
```

## Database Setup

The system uses Neon PostgreSQL for user authentication and conversation history storage.

### Initialization

Run the database initialization script to create required tables:

```bash
cd backend
python scripts/init_db.py
```

This creates:
- `users` table for user authentication data
- `conversations` table for conversation history (max 50 per user)
- `interaction_logs` table for existing interaction logging

### Migration Scripts

Alternatively, run the migration scripts directly:

```bash
# Create users table
psql -d your_database -f backend/scripts/migrations/001_create_users_table.sql

# Create conversations table
psql -d your_database -f backend/scripts/migrations/002_create_conversations_table.sql
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

- **Framework**: FastAPI for async operations, streaming support, and lifespan management
- **Validation**: Pydantic for request/response validation
- **User Authentication**: Foundation for user accounts with email/password storage (preparing for Better Auth integration)
- **Conversation Storage**: User-specific conversation history with automatic retention of last 50 conversations
- **Database**: Neon Serverless Postgres for user data, conversation history, and interaction logging
- **Connection Management**: Proper initialization and cleanup using FastAPI lifespan event handlers
- **Error Handling**: Graceful degradation when database is unavailable with fallback storage
- **Vector Store**: Qdrant Cloud (read-only access)
- **Deployment**: Vercel Python Serverless Functions

## Security

- Input validation using Pydantic schemas
- Passwords securely hashed using bcrypt
- CORS configured for secure cross-origin requests
- Environment variables for sensitive configuration
- Error messages sanitized to prevent information disclosure
- SQL injection prevention through parameterized queries
- User input sanitization to prevent XSS and other injection attacks