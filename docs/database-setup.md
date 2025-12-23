# Database Setup Guide

This guide explains how to set up and configure the database for the RAG chatbot backend.

## Overview

The system uses Neon PostgreSQL for user authentication data and conversation history storage, while maintaining the existing Qdrant vector database functionality.

## Prerequisites

- Neon PostgreSQL database instance
- Qdrant vector database (existing setup)
- Python 3.11+
- asyncpg dependency

## Environment Configuration

Add the following to your `.env` file:

```bash
# Database configuration
NEON_DATABASE_URL=your_neon_database_connection_string_here
DATABASE_URL=postgresql://username:password@host:port/database

# Qdrant configuration (existing)
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Database Initialization

### Manual Initialization

Run the database initialization script:

```bash
cd backend
python scripts/init_db.py
```

This script will create the necessary tables:
- `users` table for user authentication data
- `conversations` table for conversation history
- `interaction_logs` table for existing interaction logging

### Migration Scripts

Alternatively, you can run the migration scripts directly:

```bash
# Create users table
psql -d your_database -f backend/scripts/migrations/001_create_users_table.sql

# Create conversations table
psql -d your_database -f backend/scripts/migrations/002_create_conversations_table.sql
```

## Database Schema

### Users Table
- `id`: UUID (Primary Key, auto-generated)
- `email`: VARCHAR(255), unique
- `password_hash`: VARCHAR(255), securely hashed
- `created_at`: TIMESTAMP, default current timestamp

### Conversations Table
- `id`: UUID (Primary Key, auto-generated)
- `user_id`: UUID (Foreign Key to users table)
- `query`: TEXT, user's query/input
- `response`: TEXT, chatbot's response
- `timestamp`: TIMESTAMP, default current timestamp
- `created_at`: TIMESTAMP, default current timestamp

**Note**: The system automatically maintains only the last 50 conversations per user.

## Connection Management

### Connection Pooling

The system uses asyncpg connection pooling with:
- Minimum pool size: 1
- Maximum pool size: 10
- Command timeout: 60 seconds
- Statement cache disabled (for serverless compatibility)

### Lifespan Management

The application uses FastAPI lifespan event handlers:
- On startup: Initialize database connection pool
- On shutdown: Close all database connections

### Retry Logic

The system implements retry logic for database connections:
- Maximum retry attempts: 3
- Retry delay: 2 seconds between attempts

## Authentication Integration

**Note**: This phase sets up the database foundation for user authentication. Better Auth integration will be implemented in a future phase.

The system stores user credentials (email and securely hashed passwords) in the database, preparing for future Better Auth integration.

## Conversation History

- Each user can have a maximum of 50 conversations stored
- Older conversations are automatically removed when the limit is exceeded
- Conversations are stored with query, response, and timestamp

## Error Handling

### Database Unavailability

When the database is unavailable:

1. The application continues to operate in degraded mode
2. A fallback service stores conversation data temporarily in memory
3. All operations that require the database return appropriate errors
4. Once the database becomes available again, fallback data can be migrated

### Health Checks

The system provides database health check functionality:
- Endpoint: `GET /health` (for basic app health)
- Internal health checks for database connectivity

## API Endpoints

### Authentication Endpoints

- `POST /api/v1/auth/register` - Register a new user
- `POST /api/v1/auth/login` - Authenticate a user
- `GET /api/v1/auth/me` - Get current user (not implemented yet)

### Conversation Endpoints

- `GET /api/v1/conversations` - Get user conversations (not implemented yet)
- `POST /api/v1/conversations` - Save a conversation (not implemented yet)
- `POST /api/v1/conversations/internal-save` - Internal endpoint for chat integration

## Testing

Run the database-related tests:

```bash
# Unit tests
python -m pytest backend/tests/unit/test_auth_service.py

# Integration tests
python -m pytest backend/tests/integration/test_user_auth.py
python -m pytest backend/tests/integration/test_conversation_history.py

# Contract tests
python -m pytest backend/tests/contract/test_auth_contract.py
python -m pytest backend/tests/contract/test_conversation_contract.py
```

## Troubleshooting

### Common Issues

1. **Connection refused**: Verify your NEON_DATABASE_URL is correct
2. **Authentication failed**: Check that your database credentials are correct
3. **Tables don't exist**: Run the initialization script or migration scripts
4. **SSL issues**: Ensure your Neon database allows connections from your environment

### Logging

Check the application logs for database-related messages:
- Connection success/failure
- Query execution results
- Error conditions
- Health check results