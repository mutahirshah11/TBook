import os
import logging
import asyncpg
from dotenv import load_dotenv

load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    logger.error("DATABASE_URL not found in environment variables.")
    raise ValueError("DATABASE_URL not set")

_pool = None

async def get_db_pool():
    """
    Creates or retrieves an existing asyncpg connection pool.
    """
    global _pool
    if _pool is None:
        try:
            _pool = await asyncpg.create_pool(dsn=DATABASE_URL)
            logger.info("Database connection pool created successfully.")
        except Exception as e:
            logger.error(f"Failed to create database connection pool: {e}")
            raise
    return _pool

async def close_db_pool():
    """
    Closes the asyncpg connection pool.
    """
    global _pool
    if _pool:
        await _pool.close()
        _pool = None
        logger.info("Database connection pool closed.")
