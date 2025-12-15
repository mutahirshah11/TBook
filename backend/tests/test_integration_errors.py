import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
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


def test_validation_error_integration():
    """Test that validation errors are properly handled end-to-end"""
    with TestClient(app) as client:
        # Test with invalid data (empty content)
        invalid_request = {
            "message": {
                "content": "",  # This should fail validation
                "user_id": "test-user"
            }
        }

        response = client.post(
            "/api/v1/chat",
            json=invalid_request
        )

        # Expect validation error (422)
        assert response.status_code == 422

        # Check that the error response has the expected structure
        error_data = response.json()
        assert "error" in error_data
        assert "message" in error_data
        assert "details" in error_data


def test_agent_error_integration():
    """Test that agent errors are properly handled end-to-end"""
    with TestClient(app) as client:
        # Test data
        test_message_data = {
            "content": "Test message for error handling",
            "user_id": "error-test-user"
        }

        test_request = {
            "message": test_message_data
        }

        # Mock the agent client to raise an exception
        with patch('backend.agent_client.agent_client.run_agent') as mock_run_agent:
            mock_run_agent.side_effect = Exception("Test agent error")

            response = client.post(
                "/api/v1/chat",
                json=test_request,
                headers={"Content-Type": "application/json"}
            )

            # Should return 500 for internal server error
            assert response.status_code == 500

            # Check that the error response has the expected structure
            error_data = response.json()
            assert "error" in error_data
            assert "message" in error_data
            assert "internal_server_error" in error_data["error"]


def test_streaming_error_integration():
    """Test that streaming errors are properly handled end-to-end"""
    with TestClient(app) as client:
        # Test data
        test_message_data = {
            "content": "Test streaming error",
            "user_id": "stream-error-test"
        }

        test_request = {
            "message": test_message_data,
            "stream": True
        }

        # Mock the agent streaming function to raise an exception
        with patch('backend.agent_client.agent_client.run_agent_stream') as mock_run_agent_stream:
            # Create a generator that raises an exception
            async def mock_error_stream(message):
                yield {"content": "Starting stream", "status": "partial"}
                raise Exception("Test streaming error")

            mock_run_agent_stream.return_value = mock_error_stream(test_message_data)

            response = client.post(
                "/api/v1/chat/stream",
                json=test_request,
                headers={"Content-Type": "application/json"}
            )

            # Should return 200 but contain error information in the stream
            assert response.status_code == 200
            assert "error" in response.text.lower()


def test_long_content_error_integration():
    """Test that overly long content is properly rejected"""
    with TestClient(app) as client:
        # Test with content that exceeds maximum length (4000 chars)
        long_content = "A" * 4001  # Exceeds max length of 4000

        long_request = {
            "message": {
                "content": long_content,
                "user_id": "test-user"
            }
        }

        response = client.post(
            "/api/v1/chat",
            json=long_request
        )

        # Should return 422 for validation error
        assert response.status_code == 422

        # Check error details
        error_data = response.json()
        assert "error" in error_data
        assert "validation_error" in error_data["error"]


def test_invalid_user_id_format_integration():
    """Test that invalid user ID formats are properly rejected"""
    with TestClient(app) as client:
        # Test with invalid user ID format (contains invalid characters)
        invalid_request = {
            "message": {
                "content": "Test message",
                "user_id": "invalid user id with spaces"  # Invalid format
            }
        }

        response = client.post(
            "/api/v1/chat",
            json=invalid_request
        )

        # Should return 422 for validation error
        assert response.status_code == 422

        # Check error details
        error_data = response.json()
        assert "error" in error_data
        assert "validation_error" in error_data["error"]


def test_missing_required_fields_integration():
    """Test that requests with missing required fields are properly rejected"""
    with TestClient(app) as client:
        # Test with missing required content field
        invalid_request = {
            "message": {
                # Missing required 'content' field
                "user_id": "test-user"
            }
        }

        response = client.post(
            "/api/v1/chat",
            json=invalid_request
        )

        # Should return 422 for validation error
        assert response.status_code == 422

        # Check error details
        error_data = response.json()
        assert "error" in error_data
        assert "validation_error" in error_data["error"]