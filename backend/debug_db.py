import asyncio
import os
from dotenv import load_dotenv
import asyncpg

# Load from parent directory .env
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
env_path = os.path.join(parent_dir, '.env')
load_dotenv(env_path)

url = os.getenv("NEON_DATABASE_URL")
print(f"Loaded .env from: {env_path}")
print(f"NEON_DATABASE_URL found: {'Yes' if url else 'No'}")
if url:
    print(f"URL starts with: {url[:15]}...")

async def test_conn():
    if not url:
        print("ERROR: No NEON_DATABASE_URL")
        return
    
    print("Attempting to connect...")
    try:
        conn = await asyncpg.connect(url)
        print("SUCCESS: Connected to database!")
        await conn.close()
    except Exception as e:
        print(f"FAILURE: {e}")

if __name__ == "__main__":
    asyncio.run(test_conn())
