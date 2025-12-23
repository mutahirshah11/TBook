"""
Integration tests for user authentication functionality.
Tests the complete user registration and authentication flow.
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from main import app
from utils.database import neon_db
import uuid


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    return TestClient(app)


@pytest.mark.asyncio
async def test_user_registration_integration():
    """Test the complete user registration integration flow."""
    # This test will fail initially until the auth endpoints are implemented
    # Import here to avoid circular dependencies during test setup
    from src.services.database_service import DatabaseService
    from src.models.user import User
    from src.services.user_service import UserService

    # Create a unique email for this test
    test_email = f"test_{uuid.uuid4()}@example.com"
    test_password = "securepassword123"

    # Test user registration
    user_service = UserService()
    user_id = await user_service.register_user(test_email, test_password)

    # Verify user was created
    assert user_id is not None

    # Verify user exists in database
    db_service = DatabaseService()
    user = await db_service.fetch_one(
        "SELECT id, email, password_hash FROM users WHERE email = $1",
        test_email
    )

    assert user is not None
    assert user['email'] == test_email

    # Test user authentication
    auth_result = await user_service.authenticate_user(test_email, test_password)
    assert auth_result is not None
    assert auth_result['id'] == user['id']

    # Test with wrong password
    wrong_auth_result = await user_service.authenticate_user(test_email, "wrongpassword")
    assert wrong_auth_result is None


@pytest.mark.asyncio
async def test_duplicate_user_registration():
    """Test that duplicate user registration is handled properly."""
    # Create a unique email for this test
    test_email = f"dup_test_{uuid.uuid4()}@example.com"
    test_password = "securepassword123"

    # Register user first time
    from src.services.user_service import UserService
    user_service = UserService()
    first_registration = await user_service.register_user(test_email, test_password)
    assert first_registration is not None

    # Try to register same user again
    second_registration = await user_service.register_user(test_email, test_password)
    # Should fail due to unique constraint
    assert second_registration is None


@pytest.mark.asyncio
async def test_user_authentication_with_invalid_credentials():
    """Test user authentication with invalid credentials."""
    # Create a unique email for this test
    test_email = f"auth_test_{uuid.uuid4()}@example.com"
    test_password = "securepassword123"

    # Register user first
    from src.services.user_service import UserService
    user_service = UserService()
    user_id = await user_service.register_user(test_email, test_password)
    assert user_id is not None

    # Try to authenticate with wrong password
    auth_result = await user_service.authenticate_user(test_email, "wrongpassword")
    assert auth_result is None

    # Try to authenticate with non-existent email
    non_existent_auth = await user_service.authenticate_user("nonexistent@example.com", test_password)
    assert non_existent_auth is None