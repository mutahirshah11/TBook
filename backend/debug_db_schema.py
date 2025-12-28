import asyncio
import os
from dotenv import load_dotenv
import asyncpg

async def list_schema():
    load_dotenv()
    url = os.getenv("NEON_DATABASE_URL")
    if not url:
        print("No NEON_DATABASE_URL")
        return

    try:
        conn = await asyncpg.connect(url)
        print("--- Tables and Column Types ---")
        tables = await conn.fetch("SELECT table_name FROM information_schema.tables WHERE table_schema = 'public'")
        for t in tables:
            table_name = t['table_name']
            print(f"Table: {table_name}")
            columns = await conn.fetch(f"""
                SELECT column_name, data_type, character_maximum_length 
                FROM information_schema.columns 
                WHERE table_name = '{table_name}'
            """)
            for c in columns:
                print(f"  - {c['column_name']}: {c['data_type']} ({c['character_maximum_length']})")
        
        await conn.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(list_schema())