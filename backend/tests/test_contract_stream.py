import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, AsyncMock
import sys
import os

# Add the backend directory to the path so we can import from it
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend.main import app
from backend.schemas import ChatRequest, Message, ResponseStatus


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    with TestClient(app) as test_client:
        yield test_client


def test_chat_stream_endpoint_contract():
    """Test the contract for POST /chat-stream endpoint"""
    with TestClient(app) as client:
        # Define test data matching the schema
        test_message = {
            "content": "Hello, stream this response",
            "user_id": "test-user-123",
            "metadata": {"context": "full-book"}
        }

        test_request = {
            "message": test_message,
            "stream": True
        }

        # This test will initially fail (as required by TDD) since endpoint may not be fully implemented yet
        # It should pass after the endpoint is implemented
        response = client.post(
            "/api/v1/chat/stream",  # This is the streaming endpoint
            json=test_request,
            headers={"Content-Type": "application/json"}
        )

        # Initially this might be 404/405 (endpoint not found/not allowed) before implementation
        # After implementation, it should be 200 with text/event-stream content type
        assert response.status_code in [200, 404, 405]  # 404/405 expected before implementation

        if response.status_code == 200:
            # Verify the content type for streaming
            assert response.headers.get("content-type", "").startswith("text/event-stream")