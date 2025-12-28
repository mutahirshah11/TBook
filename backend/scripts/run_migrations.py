import asyncio
import asyncpg
import os
import logging
from pathlib import Path

async def run_migrations():
    database_url = os.getenv("NEON_DATABASE_URL") or os.getenv("DATABASE_URL")
    if not database_url:
        logging.error("DATABASE_URL environment variable not set")
        return False

    migrations_dir = Path(__file__).parent / "migrations"
    if not migrations_dir.exists():
        logging.error(f"Migrations directory not found: {migrations_dir}")
        return False

    # Get all .sql files sorted by name
    migration_files = sorted(migrations_dir.glob("*.sql"))

    connection = None
    try:
        connection = await asyncpg.connect(database_url)
        logging.info("Connected to database")

        for sql_file in migration_files:
            logging.info(f"Applying migration: {sql_file.name}")
            sql_content = sql_file.read_text()
            try:
                # Execute the SQL content
                # We split by semicolon? asyncpg execute can handle multiple statements usually if simple.
                # But better to use simple execute.
                await connection.execute(sql_content)
                logging.info(f"Successfully applied: {sql_file.name}")
            except Exception as e:
                logging.error(f"Failed to apply {sql_file.name}: {e}")
                # We might want to stop or continue? Stop is safer.
                # But if table exists, it might fail.
                # For this dev setup, we'll stop and report.
                return False

        return True

    except Exception as e:
        logging.error(f"Migration process failed: {e}")
        return False
    finally:
        if connection:
            await connection.close()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # Load env if needed (e.g. using python-dotenv)
    try:
        from dotenv import load_dotenv
        load_dotenv(Path(__file__).parents[2] / ".env")
    except ImportError:
        pass

    asyncio.run(run_migrations())
