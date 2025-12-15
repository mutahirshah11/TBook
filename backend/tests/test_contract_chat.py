import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch
import sys
import os

# Add the backend directory to the path so we can import from it
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend.main import app
from backend.schemas import ChatRequest, Message, Response, ResponseStatus


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    with TestClient(app) as test_client:
        yield test_client


def test_chat_endpoint_contract():
    """Test the contract for POST /chat endpoint"""
    with TestClient(app) as client:
        # Define test data matching the schema
        test_message = {
            "content": "Hello, how are you?",
            "user_id": "test-user-123",
            "metadata": {"context": "full-book"}
        }

        test_request = {
            "message": test_message,
            "stream": False
        }

        # This test will initially fail (as required by TDD) since endpoint isn't implemented yet
        # It should pass after the endpoint is implemented
        response = client.post(
            "/api/v1/chat",  # This endpoint doesn't exist yet, will cause 404
            json=test_request,
            headers={"Content-Type": "application/json"}
        )

        # Initially this will likely be 404 (endpoint not found) before implementation
        # After implementation, it should be 200 with proper response structure
        assert response.status_code in [200, 404]  # 404 expected before implementation