import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, AsyncMock, MagicMock
import sys
import os
import json

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


def test_streaming_response_flow_integration():
    """Test the complete streaming message flow from UI to agent and back via SSE"""
    with TestClient(app) as client:
        # Test data
        test_message_data = {
            "content": "Test streaming message for integration",
            "user_id": "integration-test-user",
            "metadata": {"context": "full-book"}
        }

        test_request = {
            "message": test_message_data,
            "stream": True
        }

        # Mock the agent client streaming function to avoid actual agent calls during testing
        async def mock_stream_generator(message):
            # Simulate streaming response
            content_parts = [
                {"content": "Processing your request", "status": "partial"},
                {"content": "Retrieving relevant information", "status": "partial"},
                {"content": "Generating response", "status": "partial"},
                {"content": "Here's the answer to your question", "status": "success"}
            ]
            for part in content_parts:
                yield part

        # Patch the agent client streaming method
        with patch('backend.agent_client.agent_client.run_agent_stream') as mock_run_agent_stream:
            mock_run_agent_stream.return_value = mock_stream_generator(test_message_data)

            # Call the streaming endpoint
            response = client.post(
                "/api/v1/chat/stream",
                json=test_request,
                headers={"Content-Type": "application/json"}
            )

            # Should return 200 with streaming content type
            assert response.status_code == 200
            assert response.headers.get("content-type", "").startswith("text/event-stream")

            # Verify that the response contains streaming data
            response_text = response.text
            assert "data:" in response_text  # SSE format should have "data:" lines

            # Check that we have multiple SSE events
            lines = response_text.strip().split('\n\n')
            sse_events = [line for line in lines if line.startswith("data:")]
            assert len(sse_events) > 0


def test_streaming_validation_integration():
    """Test that invalid streaming requests are properly rejected"""
    with TestClient(app) as client:
        # Test with invalid data (empty content)
        invalid_request = {
            "message": {
                "content": "",  # This should fail validation
                "user_id": "test-user"
            },
            "stream": True
        }

        response = client.post(
            "/api/v1/chat/stream",
            json=invalid_request
        )

        # Expect validation error (422) for invalid content
        assert response.status_code == 422


def test_streaming_error_handling_integration():
    """Test that streaming errors are properly handled"""
    with TestClient(app) as client:
        # Test data
        test_message_data = {
            "content": "Test streaming error handling",
            "user_id": "integration-test-user"
        }

        test_request = {
            "message": test_message_data,
            "stream": True
        }

        # Mock the agent client to raise an exception
        with patch('backend.agent_client.agent_client.run_agent_stream') as mock_run_agent_stream:
            # Create an async generator that raises an exception
            async def mock_stream_generator_with_error(message):
                yield {"content": "Starting stream", "status": "partial"}
                raise Exception("Test error in streaming")

            mock_run_agent_stream.return_value = mock_stream_generator_with_error(test_message_data)

            # Call the streaming endpoint
            response = client.post(
                "/api/v1/chat/stream",
                json=test_request,
                headers={"Content-Type": "application/json"}
            )

            # Should still return 200 but with error data in the stream
            assert response.status_code == 200
            assert "error" in response.text.lower() or "exception" in response.text.lower()


def test_streaming_response_format():
    """Test that streaming responses are properly formatted as Server-Sent Events"""
    with TestClient(app) as client:
        # Test data
        test_message_data = {
            "content": "Test streaming format",
            "user_id": "format-test-user"
        }

        test_request = {
            "message": test_message_data,
            "stream": True
        }

        # Mock a simple streaming response
        async def mock_simple_stream(message):
            yield {"content": "Hello", "status": "partial"}
            yield {"content": "World", "status": "success"}

        with patch('backend.agent_client.agent_client.run_agent_stream') as mock_run_agent_stream:
            mock_run_agent_stream.return_value = mock_simple_stream(test_message_data)

            response = client.post(
                "/api/v1/chat/stream",
                json=test_request,
                headers={"Content-Type": "application/json"}
            )

            assert response.status_code == 200
            response_text = response.text

            # Verify SSE format: each event should be "data: {json}\n\n"
            lines = response_text.strip().split('\n\n')
            sse_events = [line for line in lines if line.startswith("data:")]

            # Should have at least 2 events
            assert len(sse_events) >= 2

            # Each event should contain valid JSON
            for event in sse_events:
                # Extract JSON part after "data: "
                json_str = event[5:].strip()  # Remove "data: " prefix
                try:
                    parsed = json.loads(json_str)
                    assert isinstance(parsed, dict)
                    assert "content" in parsed
                    assert "status" in parsed
                except json.JSONDecodeError:
                    assert False, f"Invalid JSON in SSE event: {json_str}"