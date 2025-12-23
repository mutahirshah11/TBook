"""
Fallback service for handling operations when database is unavailable.
Provides graceful degradation for core functionality when database is down.
"""
from typing import Any, Dict, List, Optional, Callable, Awaitable
import logging
import asyncio
from datetime import datetime


class FallbackService:
    """
    Service that provides fallback functionality when database is unavailable.
    Allows the application to continue operating basic functionality while
    database issues are resolved.
    """

    def __init__(self):
        self.fallback_storage = {}  # In-memory storage for fallback data
        self.fallback_conversations = {}  # Temporary conversation storage
        self.fallback_enabled = True  # Whether fallback mode is active

    def set_fallback_mode(self, enabled: bool):
        """Enable or disable fallback mode."""
        self.fallback_enabled = enabled
        status = "enabled" if enabled else "disabled"
        logging.info(f"Fallback mode {status}")

    def is_fallback_active(self) -> bool:
        """Check if fallback mode is currently active."""
        return self.fallback_enabled

    async def store_conversation_fallback(self, user_id: str, query: str, response: str) -> bool:
        """
        Store conversation in fallback storage when database is unavailable.
        """
        if not self.fallback_enabled:
            return False

        try:
            # Create conversation record
            conversation_record = {
                'user_id': user_id,
                'query': query,
                'response': response,
                'timestamp': datetime.utcnow(),
                'id': f"fallback_{user_id}_{datetime.utcnow().timestamp()}"
            }

            # Add to user's conversation list
            if user_id not in self.fallback_conversations:
                self.fallback_conversations[user_id] = []

            # Add new conversation
            self.fallback_conversations[user_id].append(conversation_record)

            # Maintain only last 50 conversations per user (as per requirement)
            if len(self.fallback_conversations[user_id]) > 50:
                self.fallback_conversations[user_id] = self.fallback_conversations[user_id][-50:]

            logging.info(f"Stored conversation in fallback for user {user_id}")
            return True
        except Exception as e:
            logging.error(f"Failed to store conversation in fallback: {str(e)}")
            return False

    async def get_user_conversations_fallback(self, user_id: str, limit: int = 50) -> List[Dict[str, Any]]:
        """
        Retrieve conversations from fallback storage.
        """
        if not self.fallback_enabled:
            return []

        try:
            user_convs = self.fallback_conversations.get(user_id, [])
            # Return most recent conversations up to the limit
            return user_convs[-limit:] if user_convs else []
        except Exception as e:
            logging.error(f"Failed to retrieve conversations from fallback: {str(e)}")
            return []

    async def clear_fallback_conversations(self, user_id: str) -> bool:
        """
        Clear fallback conversations for a specific user.
        """
        try:
            if user_id in self.fallback_conversations:
                del self.fallback_conversations[user_id]
                logging.info(f"Cleared fallback conversations for user {user_id}")
                return True
            return True  # Already cleared
        except Exception as e:
            logging.error(f"Failed to clear fallback conversations: {str(e)}")
            return False

    async def get_fallback_stats(self) -> Dict[str, Any]:
        """
        Get statistics about fallback storage.
        """
        try:
            total_users = len(self.fallback_conversations)
            total_conversations = sum(len(convs) for convs in self.fallback_conversations.values())

            return {
                'fallback_enabled': self.fallback_enabled,
                'total_users': total_users,
                'total_conversations': total_conversations,
                'timestamp': datetime.utcnow()
            }
        except Exception as e:
            logging.error(f"Failed to get fallback stats: {str(e)}")
            return {
                'fallback_enabled': self.fallback_enabled,
                'total_users': 0,
                'total_conversations': 0,
                'timestamp': datetime.utcnow(),
                'error': str(e)
            }

    async def migrate_to_database(self, db_service) -> bool:
        """
        Migrate fallback data to the main database when it becomes available.
        This is a complex operation that would need to be customized based on the database structure.
        """
        if not self.fallback_enabled or not self.fallback_conversations:
            return True  # Nothing to migrate

        try:
            # This would involve complex logic to migrate fallback data to the main database
            # For now, we'll just log that migration is needed
            logging.info(f"Migration needed: {len(self.fallback_conversations)} users have fallback data")

            # Note: Actual implementation would iterate through fallback_conversations
            # and save each conversation using the database service
            # This is a placeholder implementation
            for user_id, conversations in self.fallback_conversations.items():
                logging.info(f"Would migrate {len(conversations)} conversations for user {user_id}")

            return True
        except Exception as e:
            logging.error(f"Failed to migrate fallback data: {str(e)}")
            return False

    async def execute_with_fallback(self,
                                  primary_func: Callable[[], Awaitable[Any]],
                                  fallback_func: Callable[[], Awaitable[Any]],
                                  operation_name: str = "operation") -> Any:
        """
        Execute a function with fallback mechanism.
        If primary function fails (e.g., due to DB issues), execute fallback function.
        """
        try:
            result = await primary_func()
            # If successful, ensure fallback mode is disabled for this operation
            if self.fallback_enabled:
                logging.info(f"{operation_name} succeeded with primary service")
            return result
        except Exception as e:
            logging.warning(f"{operation_name} failed with primary service: {str(e)}")

            if self.fallback_enabled:
                try:
                    result = await fallback_func()
                    logging.info(f"{operation_name} succeeded with fallback service")
                    return result
                except Exception as fallback_error:
                    logging.error(f"{operation_name} failed with both primary and fallback: {str(fallback_error)}")
                    raise fallback_error
            else:
                # If fallback is disabled, just raise the original error
                raise e


# Global instance of fallback service
fallback_service = FallbackService()


async def with_fallback(primary_func: Callable[[], Awaitable[Any]],
                       fallback_func: Callable[[], Awaitable[Any]],
                       operation_name: str = "operation") -> Any:
    """
    Convenience function to execute operations with fallback.
    """
    return await fallback_service.execute_with_fallback(
        primary_func, fallback_func, operation_name
    )