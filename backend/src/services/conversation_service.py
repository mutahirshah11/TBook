"""
Conversation service for handling conversation history operations.
Manages user conversation storage with automatic limit enforcement (max 50 per user).
"""
from typing import List, Optional, Dict, Any
from src.models.conversation import ConversationCreate, ConversationPublic
from utils.database import neon_db
from src.services.database_service import DatabaseService
import logging
from datetime import datetime
import uuid


class ConversationService:
    """Service class for conversation history operations."""

    # Maximum number of conversations to store per user
    MAX_CONVERSATIONS_PER_USER = 50

    def __init__(self):
        self.db_service = DatabaseService()

    async def save_conversation(self, conversation_data: ConversationCreate) -> Optional[str]:
        """
        Save a conversation to the database.
        Enforces the limit of MAX_CONVERSATIONS_PER_USER conversations per user.
        """
        try:
            # Validate the conversation data
            if not await self._validate_conversation_data(conversation_data):
                logging.warning(f"Invalid conversation data for user {conversation_data.user_id}")
                return None

            # Generate conversation ID
            conversation_id = str(uuid.uuid4())

            # Insert conversation into database
            query = """
                INSERT INTO conversations (id, user_id, query, response, timestamp, created_at)
                VALUES ($1, $2, $3, $4, $5, $6)
                RETURNING id
            """

            async with neon_db.pool.acquire() as conn:
                result = await conn.fetchval(
                    query,
                    conversation_id,
                    conversation_data.user_id,
                    conversation_data.query,
                    conversation_data.response,
                    datetime.utcnow(),
                    datetime.utcnow()
                )

                if result:
                    logging.info(f"Conversation saved for user {conversation_data.user_id}")

                    # Enforce conversation limit after saving
                    await self._enforce_conversation_limit(conversation_data.user_id)

                    return str(result)
                else:
                    logging.warning(f"Failed to save conversation for user {conversation_data.user_id}")
                    return None
        except Exception as e:
            logging.error(f"Error saving conversation for user {conversation_data.user_id}: {str(e)}")
            return None

    async def get_user_conversations(self, user_id: str, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Get conversations for a specific user, ordered by timestamp (most recent first).
        """
        try:
            # Use provided limit or default to the max limit
            effective_limit = limit or self.MAX_CONVERSATIONS_PER_USER
            effective_limit = min(effective_limit, self.MAX_CONVERSATIONS_PER_USER)  # Don't exceed max

            query = """
                SELECT id, user_id, query, response, timestamp, created_at
                FROM conversations
                WHERE user_id = $1
                ORDER BY timestamp DESC
                LIMIT $2
            """

            async with neon_db.pool.acquire() as conn:
                rows = await conn.fetch(query, user_id, effective_limit)

            # Convert asyncpg records to dictionaries
            conversations = []
            for row in rows:
                conversation = {
                    'id': str(row['id']),
                    'user_id': str(row['user_id']),
                    'query': row['query'],
                    'response': row['response'],
                    'timestamp': row['timestamp'],
                    'created_at': row['created_at']
                }
                conversations.append(conversation)

            logging.info(f"Retrieved {len(conversations)} conversations for user {user_id}")
            return conversations
        except Exception as e:
            logging.error(f"Error retrieving conversations for user {user_id}: {str(e)}")
            return []

    async def get_conversation_by_id(self, conversation_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific conversation by ID."""
        try:
            query = """
                SELECT id, user_id, query, response, timestamp, created_at
                FROM conversations
                WHERE id = $1
            """

            async with neon_db.pool.acquire() as conn:
                row = await conn.fetchrow(query, conversation_id)

            if row:
                return {
                    'id': str(row['id']),
                    'user_id': str(row['user_id']),
                    'query': row['query'],
                    'response': row['response'],
                    'timestamp': row['timestamp'],
                    'created_at': row['created_at']
                }
            return None
        except Exception as e:
            logging.error(f"Error retrieving conversation {conversation_id}: {str(e)}")
            return None

    async def delete_conversation(self, conversation_id: str) -> bool:
        """Delete a specific conversation by ID."""
        try:
            query = "DELETE FROM conversations WHERE id = $1"

            async with neon_db.pool.acquire() as conn:
                result = await conn.execute(query, conversation_id)

            success = "DELETE 0" not in result or "DELETE 1" in result
            if success:
                logging.info(f"Deleted conversation {conversation_id}")
            else:
                logging.warning(f"Failed to delete conversation {conversation_id}")
            return success
        except Exception as e:
            logging.error(f"Error deleting conversation {conversation_id}: {str(e)}")
            return False

    async def delete_user_conversations(self, user_id: str) -> bool:
        """Delete all conversations for a specific user."""
        try:
            query = "DELETE FROM conversations WHERE user_id = $1"

            async with neon_db.pool.acquire() as conn:
                result = await conn.execute(query, user_id)

            logging.info(f"Deleted all conversations for user {user_id}")
            return True
        except Exception as e:
            logging.error(f"Error deleting conversations for user {user_id}: {str(e)}")
            return False

    async def _validate_conversation_data(self, conversation_data: ConversationCreate) -> bool:
        """Validate conversation data before saving."""
        if not conversation_data:
            return False

        if not conversation_data.user_id or not conversation_data.user_id.strip():
            return False

        if not conversation_data.query or not conversation_data.query.strip():
            return False

        if not conversation_data.response or not conversation_data.response.strip():
            return False

        # Check length limits (optional but recommended)
        if len(conversation_data.query) > 10000:  # 10k characters max for query
            return False

        if len(conversation_data.response) > 10000:  # 10k characters max for response
            return False

        return True

    async def _enforce_conversation_limit(self, user_id: str) -> bool:
        """
        Enforce the conversation limit by deleting oldest conversations if needed.
        Keeps only the most recent MAX_CONVERSATIONS_PER_USER conversations per user.
        """
        try:
            # First, count the number of conversations for this user
            count_query = """
                SELECT COUNT(*) as count
                FROM conversations
                WHERE user_id = $1
            """

            async with neon_db.pool.acquire() as conn:
                count_result = await conn.fetchrow(count_query, user_id)
                current_count = count_result['count']

            # If we're under the limit, no action needed
            if current_count <= self.MAX_CONVERSATIONS_PER_USER:
                return True

            # Calculate how many conversations to delete (excess over the limit)
            conversations_to_delete = current_count - self.MAX_CONVERSATIONS_PER_USER

            # Delete the oldest conversations
            delete_query = """
                DELETE FROM conversations
                WHERE id IN (
                    SELECT id
                    FROM conversations
                    WHERE user_id = $1
                    ORDER BY timestamp ASC
                    LIMIT $2
                )
            """

            async with neon_db.pool.acquire() as conn:
                result = await conn.execute(delete_query, user_id, conversations_to_delete)

            logging.info(
                f"Enforced conversation limit for user {user_id}: "
                f"deleted {conversations_to_delete} oldest conversations"
            )
            return True
        except Exception as e:
            logging.error(f"Error enforcing conversation limit for user {user_id}: {str(e)}")
            return False

    async def get_user_conversation_count(self, user_id: str) -> int:
        """Get the count of conversations for a specific user."""
        try:
            query = "SELECT COUNT(*) as count FROM conversations WHERE user_id = $1"

            async with neon_db.pool.acquire() as conn:
                result = await conn.fetchrow(query, user_id)

            return result['count'] if result else 0
        except Exception as e:
            logging.error(f"Error getting conversation count for user {user_id}: {str(e)}")
            return 0