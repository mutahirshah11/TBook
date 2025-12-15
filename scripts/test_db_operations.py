import asyncio
import uuid
import logging
from src.database.ops import save_user_query, save_chat_history, get_chat_history
from src.database.connection import close_db_pool

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def run_full_test():
    user_id = uuid.uuid4()
    logger.info(f"Starting end-to-end DB test for user: {user_id}")

    # 1. Save Query
    await save_user_query(user_id, "Hello, Neon!")
    
    # 2. Save Chat History
    await save_chat_history(
        user_id, 
        "Who are you?", 
        "I am a robot.", 
        {"context": "robotics_intro.md"}
    )

    # Additional entry based on user request
    await save_user_query(user_id, "hello my name is mutahir")
    await save_chat_history(
        user_id,
        "hello my name is mutahir",
        "Hello Mutahir! How can I assist you today?",
        {"context": "greeting_protocol"}
    )

    # 3. Retrieve History (again)
    history_updated = await get_chat_history(user_id)
    
    if len(history_updated) > 0:
        logger.info("\nSUCCESS: Retrieved UPDATED chat history.")
        for i, entry in enumerate(history_updated):
            logger.info(f"  Entry {i+1} (Query): {entry.get('query')}")
            logger.info(f"  Entry {i+1} (Response): {entry.get('response')}")
    else:
        logger.error("FAILURE: No history retrieved after second interaction.")

    await close_db_pool()

if __name__ == "__main__":
    asyncio.run(run_full_test())
