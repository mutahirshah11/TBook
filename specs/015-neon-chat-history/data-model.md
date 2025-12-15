# Data Model: Neon Postgres Chat History

## Entities

### 1. UserQuery
Represents a single input query from a user.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| `id` | `UUID` | Primary Key. Unique identifier for the record. | Valid UUID v4. |
| `user_id` | `UUID` | Foreign Key (logical). Identifies the user who made the query. | Valid UUID. |
| `query` | `TEXT` | The actual text content of the user's question. | Not empty. |
| `timestamp` | `TIMESTAMP WITH TIME ZONE` | When the query was received. | Default: Current Time. |

### 2. ChatHistory
Represents a full interaction turn (user query + system response + context).

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| `id` | `UUID` | Primary Key. Unique identifier for the record. | Valid UUID v4. |
| `user_id` | `UUID` | Foreign Key (logical). Identifies the user. | Valid UUID. |
| `query` | `TEXT` | The user's input query. | Not empty. |
| `response` | `TEXT` | The system's generated response. | Not empty (usually). |
| `used_chunks` | `JSONB` | List of source chunks used to generate the response. | Valid JSON array/object. |
| `timestamp` | `TIMESTAMP WITH TIME ZONE` | When the interaction occurred. | Default: Current Time. |

## Schema Definition (SQL)

```sql
CREATE TABLE IF NOT EXISTS user_queries (
    id UUID PRIMARY KEY,
    user_id UUID NOT NULL,
    query TEXT NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS chat_history (
    id UUID PRIMARY KEY,
    user_id UUID NOT NULL,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    used_chunks JSONB,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);
```
