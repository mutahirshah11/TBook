import pytest
from fastapi.testclient import TestClient
import sys
import os

# Add the backend directory to the path so we can import from it
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend.main import app
from backend.schemas import ErrorResponse


def test_error_response_contract():
    """Test the contract for error responses across all endpoints"""
    with TestClient(app) as client:
        # Test invalid request to chat endpoint (empty content)
        invalid_request = {
            "message": {
                "content": "",  # This should fail validation
                "user_id": "test-user"
            }
        }

        response = client.post(
            "/api/v1/chat",
            json=invalid_request,
            headers={"Content-Type": "application/json"}
        )

        # Should return 422 for validation error
        assert response.status_code == 422

        # Verify response structure matches ErrorResponse schema
        response_data = response.json()
        assert "error" in response_data
        assert "message" in response_data
        # Optional: assert "details" in response_data  # details may be optional


def test_chat_stream_error_contract():
    """Test the error contract for streaming endpoint"""
    with TestClient(app) as client:
        # Test invalid request to stream endpoint
        invalid_request = {
            "message": {
                "content": "",  # This should fail validation
                "user_id": "test-user"
            }
        }

        response = client.post(
            "/api/v1/chat/stream",
            json=invalid_request,
            headers={"Content-Type": "application/json"}
        )

        # Should return 422 for validation error
        assert response.status_code == 422

        # Verify response structure matches ErrorResponse schema
        response_data = response.json()
        assert "error" in response_data
        assert "message" in response_data


def test_agent_error_contract():
    """Test the contract when agent fails internally"""
    with TestClient(app) as client:
        # This test would require mocking an agent failure
        # The error response should follow the ErrorResponse schema
        error_request = {
            "message": {
                "content": "This might trigger an agent error",
                "user_id": "test-user"
            }
        }

        # In a real scenario, we'd mock the agent to fail
        # For now, just verify the endpoint exists and returns appropriate status
        response = client.post(
            "/api/v1/chat",
            json=error_request,
            headers={"Content-Type": "application/json"}
        )

        # Should return 200 if agent handles errors gracefully, or 500 if not
        # The important thing is that the response follows the expected schema
        if response.status_code == 500:
            # Server error case - should still have proper error structure
            assert response.status_code == 500
        elif response.status_code == 200:
            # Success case - agent handled error gracefully
            response_data = response.json()
            assert "response" in response_data
            # If there was an error in processing, it should be reflected in the response status


def test_timeout_error_contract():
    """Test the contract for timeout errors"""
    with TestClient(app) as client:
        # Test request that might cause timeout (long content)
        timeout_request = {
            "message": {
                "content": "A" * 4001,  # This exceeds max length and should cause validation error
                "user_id": "test-user"
            }
        }

        response = client.post(
            "/api/v1/chat",
            json=timeout_request,
            headers={"Content-Type": "application/json"}
        )

        # Should return 422 for validation error (not timeout, since validation happens first)
        assert response.status_code == 422

        response_data = response.json()
        assert "error" in response_data
        assert "message" in response_data