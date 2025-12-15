import asyncio
import uuid
import logging
from src.database.ops import save_chat_history
from src.database.connection import close_db_pool

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_save_chat_history():
    test_user_id = uuid.uuid4()
    test_query = "Explain quantum computing."
    test_response = "Quantum computing uses qubits..."
    test_used_chunks = [
        {"chunk_id": "123", "content": "Quantum physics basics..."},
        {"chunk_id": "456", "content": "Superposition and entanglement..."}
    ]
    
    logger.info(f"Testing save_chat_history with user_id={test_user_id}")
    
    await save_chat_history(test_user_id, test_query, test_response, test_used_chunks)
    
    logger.info("Test execution finished.")
    await close_db_pool()

if __name__ == "__main__":
    asyncio.run(test_save_chat_history())
