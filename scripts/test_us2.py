import asyncio
import uuid
import logging
from src.database.ops import save_user_query
from src.database.connection import close_db_pool

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_save_user_query():
    test_user_id = uuid.uuid4()
    test_query = "What is the meaning of life?"
    
    logger.info(f"Testing save_user_query with user_id={test_user_id} and query='{test_query}'")
    
    await save_user_query(test_user_id, test_query)
    
    logger.info("Test execution finished.")
    await close_db_pool()

if __name__ == "__main__":
    asyncio.run(test_save_user_query())
