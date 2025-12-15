import pytest
import asyncio
from unittest.mock import AsyncMock, patch
import sys
import os

# Add the backend directory to the path so we can import from it
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from backend.agent_client import AgentClient
from backend.schemas import Message, Response, ResponseStatus
from datetime import datetime


@pytest.fixture
def agent_client():
    """Create an AgentClient instance for testing"""
    return AgentClient()


@pytest.fixture
def sample_message():
    """Create a sample message for testing"""
    return Message(
        content="Test message content",
        user_id="test-user-123",
        timestamp=datetime.now()
    )


def test_agent_client_initialization(agent_client):
    """Test that AgentClient initializes correctly"""
    assert agent_client.timeout is not None
    assert isinstance(agent_client.timeout, int)


@pytest.mark.asyncio
async def test_run_agent_success(agent_client, sample_message):
    """Test successful execution of run_agent"""
    # Mock the internal agent function
    with patch.object(agent_client, '_run_agent_internal', new_callable=AsyncMock) as mock_internal:
        mock_internal.return_value = "Mocked agent response"

        response = await agent_client.run_agent(sample_message)

        assert isinstance(response, Response)
        assert response.status == ResponseStatus.SUCCESS
        assert "Mocked agent response" in response.content
        assert response.message_id == sample_message.user_id


@pytest.mark.asyncio
async def test_run_agent_timeout(agent_client, sample_message):
    """Test timeout handling in run_agent"""
    # Mock the internal agent function to take longer than timeout
    async def slow_internal(message):
        await asyncio.sleep(2)  # This should exceed the timeout
        return "This should not be reached"

    with patch.object(agent_client, '_run_agent_internal', side_effect=slow_internal):
        response = await agent_client.run_agent(sample_message)

        assert isinstance(response, Response)
        assert response.status == ResponseStatus.ERROR
        assert "timeout" in response.content.lower()


@pytest.mark.asyncio
async def test_run_agent_exception(agent_client, sample_message):
    """Test exception handling in run_agent"""
    # Mock the internal agent function to raise an exception
    with patch.object(agent_client, '_run_agent_internal', side_effect=Exception("Test error")):
        response = await agent_client.run_agent(sample_message)

        assert isinstance(response, Response)
        assert response.status == ResponseStatus.ERROR
        assert "error occurred while processing" in response.content.lower()


@pytest.mark.asyncio
async def test_run_agent_stream_success(agent_client, sample_message):
    """Test successful streaming with run_agent_stream"""
    chunks = []
    async for chunk in agent_client.run_agent_stream(sample_message):
        chunks.append(chunk)

    assert len(chunks) > 0
    assert all('content' in chunk for chunk in chunks)
    assert all('status' in chunk for chunk in chunks)

    # Check that the last chunk has success status
    assert chunks[-1]['status'] in [ResponseStatus.SUCCESS.value, ResponseStatus.PARTIAL.value]


@pytest.mark.asyncio
async def test_run_agent_stream_timeout(agent_client, sample_message):
    """Test timeout handling in run_agent_stream"""
    # Temporarily reduce timeout for testing
    original_timeout = agent_client.timeout
    agent_client.timeout = 0.1  # Very short timeout for testing

    try:
        chunks = []
        async for chunk in agent_client.run_agent_stream(sample_message):
            chunks.append(chunk)
            if chunk['status'] == ResponseStatus.ERROR.value:
                break

        # Should have received an error chunk due to timeout
        if chunks:
            assert chunks[-1]['status'] == ResponseStatus.ERROR.value
            assert 'timeout' in chunks[-1]['content'].lower()
    finally:
        # Restore original timeout
        agent_client.timeout = original_timeout


@pytest.mark.asyncio
async def test_run_agent_stream_exception(agent_client, sample_message):
    """Test exception handling in run_agent_stream"""
    # Temporarily replace the method to simulate an exception
    original_method = agent_client._run_agent_internal

    async def mock_internal(message):
        raise Exception("Test stream error")

    agent_client._run_agent_internal = mock_internal

    try:
        chunks = []
        async for chunk in agent_client.run_agent_stream(sample_message):
            chunks.append(chunk)
            if chunk['status'] == ResponseStatus.ERROR.value:
                break

        # Should have received an error chunk
        assert len(chunks) >= 1
        assert chunks[-1]['status'] == ResponseStatus.ERROR.value
        assert 'error occurred while processing' in chunks[-1]['content'].lower()
    finally:
        # Restore original method
        agent_client._run_agent_internal = original_method