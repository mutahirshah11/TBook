import asyncio
import logging
from src.database.connection import get_db_pool, close_db_pool

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def setup_database():
    """
    Connects to the database and creates required tables.
    """
    pool = await get_db_pool()
    
    create_user_queries_table_sql = """
    CREATE TABLE IF NOT EXISTS user_queries (
        id UUID PRIMARY KEY,
        user_id UUID NOT NULL,
        query TEXT NOT NULL,
        timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
    );
    """

    create_chat_history_table_sql = """
    CREATE TABLE IF NOT EXISTS chat_history (
        id UUID PRIMARY KEY,
        user_id UUID NOT NULL,
        query TEXT NOT NULL,
        response TEXT NOT NULL,
        used_chunks JSONB,
        timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
    );
    """

    async with pool.acquire() as connection:
        try:
            async with connection.transaction():
                await connection.execute(create_user_queries_table_sql)
                logger.info("Table 'user_queries' created or verified.")
                
                await connection.execute(create_chat_history_table_sql)
                logger.info("Table 'chat_history' created or verified.")
                
        except Exception as e:
            logger.error(f"Error setting up database schema: {e}")
            raise
    
    await close_db_pool()

if __name__ == "__main__":
    asyncio.run(setup_database())
