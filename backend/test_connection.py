import sys
import os
import asyncio
import logging

# Setup path like main.py
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

# Configure logging
logging.basicConfig(level=logging.INFO)

async def test_startup():
    print("--- STARTING TEST ---")
    
    # Check .env manually first
    from dotenv import load_dotenv, dotenv_values
    load_dotenv()
    vals = dotenv_values()
    print(f"DATABASE_URL present? {'DATABASE_URL' in vals}")
    print(f"NEON_DATABASE_URL present? {'NEON_DATABASE_URL' in vals}")
    
    # Import database module
    try:
        from utils.database import neon_db, init_database
        print("Imported neon_db successfully")
    except ImportError as e:
        print(f"Import failed: {e}")
        return

    # Try connection
    print("Calling init_database()...")
    await init_database()
    
    if neon_db.pool:
        print("SUCCESS: Pool created!")
        await neon_db.disconnect()
    else:
        print("FAILURE: Pool is None.")

if __name__ == "__main__":
    asyncio.run(test_startup())
