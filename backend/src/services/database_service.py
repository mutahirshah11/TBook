"""
Base database service for Neon database operations.
Provides common database functionality for user authentication and conversation history.
"""
from typing import Optional, List, Dict, Any
import asyncpg
from utils.database import neon_db
import logging


class DatabaseService:
    """Base database service providing common database operations."""

    def __init__(self):
        self.db_pool = neon_db.pool

    async def get_connection(self):
        """Get a database connection from the pool."""
        if not self.db_pool:
            raise Exception("Database connection pool not available")
        return self.db_pool

    async def execute_query(self, query: str, *args) -> Optional[List[Dict[str, Any]]]:
        """Execute a SELECT query and return results."""
        if not self.db_pool:
            logging.error("Database connection pool not available")
            return None

        try:
            async with self.db_pool.acquire() as conn:
                rows = await conn.fetch(query, *args)
                # Convert asyncpg records to dictionaries
                return [dict(row) for row in rows]
        except Exception as e:
            logging.error(f"Query execution failed: {str(e)}")
            return None

    async def execute_command(self, command: str, *args) -> bool:
        """Execute an INSERT, UPDATE, or DELETE command."""
        if not self.db_pool:
            logging.error("Database connection pool not available")
            return False

        try:
            async with self.db_pool.acquire() as conn:
                await conn.execute(command, *args)
                return True
        except Exception as e:
            logging.error(f"Command execution failed: {str(e)}")
            return False

    async def fetch_one(self, query: str, *args) -> Optional[Dict[str, Any]]:
        """Execute a SELECT query and return the first result."""
        results = await self.execute_query(query, *args)
        return results[0] if results else None

    async def fetch_val(self, query: str, *args) -> Any:
        """Execute a SELECT query and return a single value."""
        if not self.db_pool:
            logging.error("Database connection pool not available")
            return None

        try:
            async with self.db_pool.acquire() as conn:
                return await conn.fetchval(query, *args)
        except Exception as e:
            logging.error(f"Fetch value failed: {str(e)}")
            return None

    async def health_check(self) -> Dict[str, Any]:
        """Perform a health check on the database service."""
        if not self.db_pool:
            return {"status": "unavailable", "message": "No database connection pool"}

        try:
            # Try to acquire a connection from the pool to test connectivity
            async with self.db_pool.acquire() as conn:
                result = await conn.fetchval("SELECT 1")
                if result == 1:
                    return {"status": "healthy", "message": "Database service is healthy"}
                else:
                    return {"status": "unhealthy", "message": "Database query returned unexpected result"}
        except Exception as e:
            return {"status": "unhealthy", "message": f"Database service health check failed: {str(e)}"}