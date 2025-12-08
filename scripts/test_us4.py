import asyncio
import uuid
import logging
from src.database.ops import save_chat_history, get_chat_history
from src.database.connection import close_db_pool

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_get_chat_history():
    test_user_id = uuid.uuid4()
    
    # Seed some data
    await save_chat_history(test_user_id, "Q1", "A1", {"source": "doc1"})
    await save_chat_history(test_user_id, "Q2", "A2", {"source": "doc2"})
    
    logger.info(f"Testing get_chat_history for user_id={test_user_id}")
    
    history = await get_chat_history(test_user_id)
    
    logger.info(f"Retrieved {len(history)} records.")
    for record in history:
        logger.info(f" - Query: {record.get('query')}, Response: {record.get('response')}")
    
    await close_db_pool()

if __name__ == "__main__":
    asyncio.run(test_get_chat_history())
