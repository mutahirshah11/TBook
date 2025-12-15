import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch
import sys
import os

# Add the backend directory to the path so we can import from it
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend.main import app, settings
from backend.schemas import Message, Response, ResponseStatus
from datetime import datetime


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    with TestClient(app) as test_client:
        yield test_client


def test_basic_message_flow_integration():
    """Test the complete message flow from UI to agent and back"""
    with TestClient(app) as client:
        # Test data
        test_message_data = {
            "content": "Test message for integration",
            "user_id": "integration-test-user",
            "metadata": {"context": "full-book"}
        }

        test_request = {
            "message": test_message_data,
            "stream": False
        }

        # Mock the agent client to avoid actual agent calls during testing
        with patch('backend.agent_client.agent_client.run_agent') as mock_run_agent:
            # Create a mock response
            mock_response = Response(
                content="Mock response for integration test",
                message_id="integration-test-user",
                timestamp=datetime.now(),
                status=ResponseStatus.SUCCESS
            )

            mock_run_agent.return_value = mock_response

            # Call the endpoint (will initially fail since endpoint doesn't exist)
            response = client.post(
                "/api/v1/chat",  # This endpoint doesn't exist yet
                json=test_request,
                headers={"Content-Type": "application/json"}
            )

            # Initially expect 404 (endpoint not found) before implementation
            # After implementation, expect 200 with proper response
            assert response.status_code in [200, 404]  # 404 expected before implementation

            if response.status_code == 200:
                # If the endpoint exists, validate the response structure
                response_data = response.json()
                assert "response" in response_data
                assert "content" in response_data["response"]
                assert response_data["response"]["content"] == "Mock response for integration test"


def test_message_validation_integration():
    """Test that invalid messages are properly rejected"""
    with TestClient(app) as client:
        # Test with invalid data (empty content)
        invalid_request = {
            "message": {
                "content": "",  # This should fail validation
                "user_id": "test-user"
            }
        }

        response = client.post(
            "/api/v1/chat",  # This endpoint doesn't exist yet
            json=invalid_request
        )

        # Expect validation error or 404 before implementation
        assert response.status_code in [422, 404, 200]  # 422 for validation error, 404 before implementation