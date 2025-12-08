import uuid
import logging
from typing import Dict, List, Any, Optional
from src.database.connection import get_db_pool

logger = logging.getLogger(__name__)

async def save_user_query(user_id: uuid.UUID, query: str) -> None:
    """
    Saves a user's query to the database.
    """
    query_id = uuid.uuid4()
    sql = """
    INSERT INTO user_queries (id, user_id, query)
    VALUES ($1, $2, $3)
    """
    pool = await get_db_pool()
    async with pool.acquire() as connection:
        try:
            await connection.execute(sql, query_id, user_id, query)
            logger.info(f"User query saved successfully: {query_id}")
        except Exception as e:
            logger.error(f"Failed to save user query: {e}")
            # Depending on requirements, we might want to re-raise or just log
            # For now, we just log as per spec "Logs error on failure"

async def save_chat_history(
    user_id: uuid.UUID, 
    query: str, 
    response: str, 
    used_chunks: Optional[Dict[str, Any] | List[Any]]
) -> None:
    """
    Saves a full chat interaction to the database.
    """
    chat_id = uuid.uuid4()
    # Ensure used_chunks is JSON serializable (asyncpg requires string input for JSONB)
    import json
    json_used_chunks = json.dumps(used_chunks) if used_chunks is not None else None
    
    sql = """
    INSERT INTO chat_history (id, user_id, query, response, used_chunks)
    VALUES ($1, $2, $3, $4, $5)
    """
    pool = await get_db_pool()
    async with pool.acquire() as connection:
        try:
            await connection.execute(sql, chat_id, user_id, query, response, json_used_chunks)
            logger.info(f"Chat history saved successfully: {chat_id}")
        except Exception as e:
            logger.error(f"Failed to save chat history: {e}")

async def get_chat_history(user_id: uuid.UUID, limit: int = 50) -> List[Dict[str, Any]]:
    """
    Retrieves chat history for a specific user.
    """
    sql = """
    SELECT id, user_id, query, response, used_chunks, timestamp
    FROM chat_history
    WHERE user_id = $1
    ORDER BY timestamp DESC
    LIMIT $2
    """
    pool = await get_db_pool()
    async with pool.acquire() as connection:
        try:
            rows = await connection.fetch(sql, user_id, limit)
            # Convert asyncpg Record objects to dictionaries
            return [dict(row) for row in rows]
        except Exception as e:
            logger.error(f"Failed to retrieve chat history: {e}")
            return []
