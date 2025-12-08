# Database Helper Functions API

These functions will be implemented in a Python module (e.g., `src/database/db_utils.py`) to abstract database interactions.

## 1. save_user_query

**Purpose**: Logs a user's input query.

**Signature**:
```python
async def save_user_query(user_id: uuid.UUID, query: str) -> None
```

**Parameters**:
- `user_id`: The UUID of the user.
- `query`: The text string of the user's question.

**Returns**: `None` (Logs error on failure).

## 2. save_chat_history

**Purpose**: Saves a full conversation turn.

**Signature**:
```python
async def save_chat_history(
    user_id: uuid.UUID, 
    query: str, 
    response: str, 
    used_chunks: dict | list
) -> None
```

**Parameters**:
- `user_id`: The UUID of the user.
- `query`: The user's input.
- `response`: The agent's output.
- `used_chunks`: Dictionary or List containing retrieval context (to be stored as JSONB).

**Returns**: `None` (Logs error on failure).

## 3. get_chat_history

**Purpose**: Retrieves past interactions for a user.

**Signature**:
```python
async def get_chat_history(user_id: uuid.UUID, limit: int = 50) -> List[Dict[str, Any]]
```

**Parameters**:
- `user_id`: The UUID of the user.
- `limit`: (Optional) Max number of recent records to retrieve.

**Returns**: A list of dictionaries, where each dictionary represents a row from `chat_history` (keys: id, query, response, timestamp, etc.). Returns empty list on error or no data.
