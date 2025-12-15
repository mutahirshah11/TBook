import sys
import os
# Add the backend directory to the path to allow imports
backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, backend_dir)

import asyncpg
import asyncio
from typing import Optional
import logging
from datetime import datetime
import json
import os
from schemas import InteractionLog


class NeonDatabase:
    """
    Database connection manager for Neon Serverless Postgres
    Used for logging interaction metadata
    """

    def __init__(self):
        self.pool: Optional[asyncpg.Pool] = None
        self.connection_string = os.getenv("NEON_DATABASE_URL", "")
        logging.info("NeonDatabase initialized with connection string")

    async def connect(self):
        """
        Establish connection to Neon database
        """
        if not self.connection_string:
            logging.warning("No Neon database URL provided, logging to database will be disabled")
            return

        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.connection_string,
                min_size=1,
                max_size=10,
                command_timeout=60,
                statement_cache_size=0  # Disable statement cache for serverless
            )
            logging.info("Successfully connected to Neon database")
        except Exception as e:
            logging.error(f"Failed to connect to Neon database: {str(e)}")
            self.pool = None

    async def disconnect(self):
        """
        Close connection to Neon database
        """
        if self.pool:
            await self.pool.close()
            logging.info("Neon database connection closed")

    async def log_interaction(self, interaction: InteractionLog):
        """
        Log interaction metadata to Neon database
        """
        if not self.pool:
            logging.warning("No database connection available for logging")
            return

        try:
            # Convert datetime objects to ISO format strings for JSON serialization
            interaction_dict = interaction.dict()

            # Prepare the data for insertion
            async with self.pool.acquire() as conn:
                await conn.execute(
                    """
                    INSERT INTO interaction_logs (
                        id, user_id, session_id, request_content, response_content,
                        request_timestamp, response_timestamp, response_time_ms,
                        agent_model, error_occurred, error_message, metadata
                    ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12)
                    """,
                    interaction_dict.get('id'),
                    interaction_dict.get('user_id'),
                    interaction_dict.get('session_id'),
                    interaction_dict.get('request_content'),
                    interaction_dict.get('response_content'),
                    interaction_dict.get('request_timestamp'),
                    interaction_dict.get('response_timestamp'),
                    interaction_dict.get('response_time_ms'),
                    interaction_dict.get('agent_model'),
                    interaction_dict.get('error_occurred'),
                    interaction_dict.get('error_message'),
                    json.dumps(interaction_dict.get('metadata')) if interaction_dict.get('metadata') else None
                )
            logging.info(f"Interaction logged successfully: {interaction_dict.get('id')}")
        except Exception as e:
            logging.error(f"Failed to log interaction to Neon database: {str(e)}")

    async def create_logs_table(self):
        """
        Create the interaction_logs table if it doesn't exist
        """
        if not self.pool:
            logging.warning("No database connection available to create table")
            return

        try:
            async with self.pool.acquire() as conn:
                await conn.execute("""
                    CREATE TABLE IF NOT EXISTS interaction_logs (
                        id TEXT PRIMARY KEY,
                        user_id TEXT,
                        session_id TEXT,
                        request_content TEXT NOT NULL,
                        response_content TEXT,
                        request_timestamp TIMESTAMP NOT NULL,
                        response_timestamp TIMESTAMP,
                        response_time_ms INTEGER,
                        agent_model TEXT,
                        error_occurred BOOLEAN DEFAULT FALSE,
                        error_message TEXT,
                        metadata JSONB
                    )
                """)
            logging.info("Interaction logs table created or already exists")
        except Exception as e:
            logging.error(f"Failed to create interaction_logs table: {str(e)}")


# Global instance
neon_db = NeonDatabase()


async def init_database():
    """
    Initialize the database connection
    """
    await neon_db.connect()
    await neon_db.create_logs_table()


async def close_database():
    """
    Close the database connection
    """
    await neon_db.disconnect()