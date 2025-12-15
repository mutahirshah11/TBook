# Quickstart: Neon Database Setup & Test

## Prerequisites

1.  **Python Environment**:
    ```bash
    pip install -r requirements.txt
    ```
    (Requires `asyncpg`, `python-dotenv`)

2.  **Environment Variables**:
    Create `.env` in the root directory:
    ```env
    DATABASE_URL=postgres://user:password@host:port/dbname?sslmode=require
    ```

## Workflow

### 1. Initialize Database
Run the setup script to create the required tables.
```bash
python scripts/setup_db.py
```
*Expected Output*: "Tables created successfully."

### 2. Run Verification Tests
Execute the test script to simulate saving and retrieving data.
```bash
python scripts/test_db_operations.py
```
*Expected Output*:
- "Connection successful."
- "User query saved."
- "Chat history saved."
- "Retrieved [X] chat records."

## Troubleshooting

-   **Connection Error**: Check `DATABASE_URL` in `.env`. Ensure your IP is allowed if Neon has restrictions (usually open for serverless).
-   **Missing Driver**: Ensure `asyncpg` is installed.
