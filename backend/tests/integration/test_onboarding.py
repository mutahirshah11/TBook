import pytest
from httpx import AsyncClient
from backend.main import app
import uuid
from backend.utils.database import neon_db
from datetime import datetime, timedelta
import asyncio

pytest_plugins = ('pytest_asyncio',)

# Setup DB connection for the module
@pytest.fixture(scope="module")
def event_loop():
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()

@pytest.fixture(scope="module")
async def db_connection():
    await neon_db.connect()
    yield neon_db
    await neon_db.disconnect()

@pytest.fixture
async def test_user(db_connection):
    pool = neon_db.pool
    user_id = str(uuid.uuid4())
    email = f"test_{user_id}@example.com"
    
    # Create user
    await pool.execute("""
        INSERT INTO "user" (id, name, email, "emailVerified", "createdAt", "updatedAt")
        VALUES ($1, 'Test User', $2, FALSE, NOW(), NOW())
    """, user_id, email)
    
    # Create session
    session_token = f"token_{user_id}"
    await pool.execute("""
        INSERT INTO "session" (id, "userId", token, "expiresAt", "createdAt", "updatedAt")
        VALUES ($1, $2, $3, $4, NOW(), NOW())
    """, str(uuid.uuid4()), user_id, session_token, datetime.now() + timedelta(hours=1))
    
    yield {"user_id": user_id, "token": session_token}
    
    # Cleanup
    await pool.execute('DELETE FROM "user" WHERE id = $1', user_id)

@pytest.mark.asyncio
async def test_onboarding_flow(test_user):
    token = test_user["token"]
    
    async with AsyncClient(app=app, base_url="http://test") as ac:
        # Set cookie
        cookies = {"better-auth.session_token": token}
        
        # Payload
        payload = {
            "python_proficiency": "Beginner",
            "developer_role": "Backend Developer"
        }
        
        response = await ac.post("/api/profile/onboarding", json=payload, cookies=cookies)
        
    assert response.status_code == 200
    assert response.json()["status"] == "success"
    
    # Verify DB
    pool = neon_db.pool
    row = await pool.fetchrow("SELECT * FROM user_profile WHERE user_id = $1", test_user["user_id"])
    assert row is not None
    assert row["python_proficiency"] == "Beginner"
    assert row["developer_role"] == "Backend Developer"
    assert row["is_onboarded"] is True

@pytest.mark.asyncio
async def test_onboarding_immutability(test_user):
    token = test_user["token"]
    
    async with AsyncClient(app=app, base_url="http://test") as ac:
        cookies = {"better-auth.session_token": token}
        
        # First onboarding
        payload = {
            "python_proficiency": "Advanced",
            "developer_role": "Frontend Developer"
        }
        await ac.post("/api/profile/onboarding", json=payload, cookies=cookies)
        
        # Second attempt should fail (403)
        response = await ac.post("/api/profile/onboarding", json=payload, cookies=cookies)
        
    assert response.status_code == 403
