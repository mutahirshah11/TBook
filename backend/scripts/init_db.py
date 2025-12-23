"""
Database initialization script for Neon database setup.
This script creates the necessary tables for user authentication and conversation history.
"""
import asyncio
import asyncpg
import os
import logging
from typing import Optional


async def create_users_table(connection):
    """Create the users table for storing user authentication data."""
    await connection.execute("""
        CREATE TABLE IF NOT EXISTS users (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            email VARCHAR(255) UNIQUE NOT NULL,
            password_hash VARCHAR(255) NOT NULL,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)
    logging.info("Users table created or already exists")


async def create_conversations_table(connection):
    """Create the conversations table for storing conversation history."""
    await connection.execute("""
        CREATE TABLE IF NOT EXISTS conversations (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            user_id UUID REFERENCES users(id) ON DELETE CASCADE,
            query TEXT NOT NULL,
            response TEXT NOT NULL,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)
    # Create index for efficient user conversation lookup
    await connection.execute("""
        CREATE INDEX IF NOT EXISTS idx_conversations_user_id ON conversations(user_id)
    """)
    # Create index for conversation ordering by time
    await connection.execute("""
        CREATE INDEX IF NOT EXISTS idx_conversations_timestamp ON conversations(timestamp)
    """)
    logging.info("Conversations table created or already exists")


async def create_interaction_logs_table(connection):
    """Create the interaction_logs table if it doesn't exist (existing table)."""
    await connection.execute("""
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


async def init_database():
    """Initialize the database with required tables."""
    database_url = os.getenv("NEON_DATABASE_URL")
    if not database_url:
        logging.error("NEON_DATABASE_URL environment variable not set")
        return False

    connection = None
    try:
        connection = await asyncpg.connect(database_url)
        logging.info("Connected to database successfully")

        # Create all required tables
        await create_users_table(connection)
        await create_conversations_table(connection)
        await create_interaction_logs_table(connection)

        logging.info("Database initialization completed successfully")
        return True
    except Exception as e:
        logging.error(f"Database initialization failed: {str(e)}")
        return False
    finally:
        if connection:
            await connection.close()


if __name__ == "__main__":
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )

    # Run the database initialization
    success = asyncio.run(init_database())
    if success:
        print("Database initialization completed successfully")
    else:
        print("Database initialization failed")
        exit(1)