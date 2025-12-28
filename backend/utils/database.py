import sys
import os
# Add the backend directory to the path to allow imports
backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, backend_dir)

import asyncpg
import asyncio
from typing import Optional
import logging
from datetime import datetime, timezone
import json
import os
from dotenv import load_dotenv
from schemas import InteractionLog

# Load environment variables to ensure NEON_DATABASE_URL is available
load_dotenv()


class NeonDatabase:
    """
    Database connection manager for Neon Serverless Postgres
    Used for logging interaction metadata
    """

    def __init__(self):
        self.pool: Optional[asyncpg.Pool] = None
        self.connection_string = ""  # Will be set in connect method
        self.max_retries = 3
        self.retry_delay = 2  # seconds
        logging.info("NeonDatabase initialized")

    async def connect_with_retry(self):
        """
        Establish connection to Neon database with retry logic
        """
        # Explicitly load .env from parent directory if variables are missing
        if not os.getenv("NEON_DATABASE_URL") and not os.getenv("DATABASE_URL"):
            parent_env = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), '.env')
            logging.info(f"Attempting to load .env from: {parent_env}")
            load_dotenv(parent_env)

        # Get the connection string from environment when connecting (not at init time)
        # Fallback to DATABASE_URL if NEON_DATABASE_URL is not found
        neon_url = os.getenv("NEON_DATABASE_URL")
        db_url = os.getenv("DATABASE_URL")
        
        self.connection_string = neon_url or db_url or ""
        
        # Log which one was found (masked)
        if neon_url:
            logging.info("Found NEON_DATABASE_URL")
        if db_url:
            logging.info("Found DATABASE_URL")

        if not self.connection_string:
            logging.warning("No Neon database URL provided (checked NEON_DATABASE_URL and DATABASE_URL), logging to database will be disabled")
            return

        for attempt in range(self.max_retries + 1):
            try:
                self.pool = await asyncpg.create_pool(
                    dsn=self.connection_string,
                    min_size=1,
                    max_size=10,
                    command_timeout=60,
                    statement_cache_size=0  # Disable statement cache for serverless
                )
                logging.info("Successfully connected to Neon database")
                return  # Success, exit the retry loop
            except Exception as e:
                logging.error(f"Attempt {attempt + 1} to connect to Neon database failed: {str(e)}")
                if attempt < self.max_retries:
                    logging.info(f"Retrying in {self.retry_delay} seconds...")
                    await asyncio.sleep(self.retry_delay)
                else:
                    logging.error("All retry attempts failed. Database connection unavailable.")
                    self.pool = None

    async def connect(self):
        """
        Establish connection to Neon database (with retry logic)
        """
        await self.connect_with_retry()

    async def reconnect(self):
        """
        Reconnect to the database with retry logic
        """
        # Close existing pool if it exists
        if self.pool:
            await self.pool.close()
            self.pool = None

        # Attempt to reconnect
        await self.connect_with_retry()

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
            # Convert datetime objects to ensure they're compatible with PostgreSQL
            interaction_dict = interaction.dict()

            # Handle datetime objects to ensure they're in the right format for PostgreSQL
            request_timestamp = interaction_dict.get('request_timestamp')
            response_timestamp = interaction_dict.get('response_timestamp')

            # Convert datetime objects to timezone-naive if they have timezone info
            if request_timestamp:
                if hasattr(request_timestamp, 'tzinfo') and request_timestamp.tzinfo is not None:
                    # Convert to UTC and make timezone-naive
                    request_timestamp = request_timestamp.astimezone(timezone.utc).replace(tzinfo=None)
                interaction_dict['request_timestamp'] = request_timestamp

            if response_timestamp:
                if hasattr(response_timestamp, 'tzinfo') and response_timestamp.tzinfo is not None:
                    # Convert to UTC and make timezone-naive
                    response_timestamp = response_timestamp.astimezone(timezone.utc).replace(tzinfo=None)
                interaction_dict['response_timestamp'] = response_timestamp

            # Prepare the data for insertion
            async with self.pool.acquire() as conn:
                await conn.execute(
                    """
                    INSERT INTO interaction_logs (
                        id, user_id, session_id, request_content, response_content,
                        request_timestamp, response_timestamp, response_time_ms,
                        agent_model, error_occurred, error_message, metadata
                    ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12)
                    ON CONFLICT (id) DO NOTHING
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
            # Check if it's a duplicate key error - if so, log as info instead of error
            error_str = str(e)
            if "duplicate key value violates unique constraint" in error_str and "interaction_logs_pkey" in error_str:
                logging.info(f"Interaction already exists, skipping: {interaction_dict.get('id')}")
            else:
                logging.error(f"Failed to log interaction to Neon database: {error_str}")

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


async def health_check():
    """
    Perform a health check on the database connection
    """
    if not neon_db.pool:
        return {"status": "unavailable", "message": "No database connection pool"}

    try:
        # Try to acquire a connection from the pool to test connectivity
        async with neon_db.pool.acquire() as conn:
            result = await conn.fetchval("SELECT 1")
            if result == 1:
                return {"status": "healthy", "message": "Database connection is healthy"}
            else:
                return {"status": "unhealthy", "message": "Database query returned unexpected result"}
    except Exception as e:
        return {"status": "unhealthy", "message": f"Database health check failed: {str(e)}"}