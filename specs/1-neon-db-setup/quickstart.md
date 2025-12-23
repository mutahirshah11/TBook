# Quickstart: Database Connection and User Data Storage

## Prerequisites
- Python 3.11+
- PostgreSQL-compatible database (Neon recommended)
- Qdrant vector database (existing setup)
- Poetry or pip for dependency management

## Setup Steps

### 1. Install Dependencies
```bash
# Add to requirements.txt or pyproject.toml
asyncpg  # For PostgreSQL connection pooling
```

### 2. Environment Configuration
```bash
# Add to .env file
DATABASE_URL=postgresql://username:password@host:port/database
NEON_DATABASE_URL=your_neon_connection_string
```

### 3. Database Tables Setup
Run the following SQL to create required tables:

```sql
-- Users table (prepared for Better Auth integration)
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Conversations table
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Index for efficient user conversation lookup
CREATE INDEX idx_conversations_user_id ON conversations(user_id);

-- Index for conversation ordering by time
CREATE INDEX idx_conversations_timestamp ON conversations(timestamp);
```

### 4. FastAPI Lifespan Integration
The application will initialize database connection pool on startup and close it on shutdown using FastAPI lifespan event handlers.

### 5. Automatic Conversation Limit
The system will maintain only the last 50 conversations per user through application-level logic that removes older entries when the limit is exceeded.

## Development Workflow
1. Start with the existing NeonDatabase class in `backend/utils/database.py`
2. Add lifespan handlers to `main.py`
3. Implement foundational user data storage (preparation for Better Auth)
4. Update conversation storage to use database instead of basic logging
5. Add tests to verify database operations
6. Verify graceful degradation when database is unavailable