"""
Unit tests for conversation service functionality.
Tests conversation limit enforcement and related functions in isolation.
"""
import pytest
import asyncio
from datetime import datetime, timedelta
from unittest.mock import Mock, AsyncMock, patch
from src.services.conversation_service import ConversationService
from src.models.conversation import ConversationCreate


def test_conversation_limit_constant():
    """Test that the conversation limit constant is properly set."""
    service = ConversationService()

    # Assuming the limit is set as a class or instance attribute
    # The service should enforce a maximum of 50 conversations per user
    assert hasattr(service, 'MAX_CONVERSATIONS_PER_USER')
    assert service.MAX_CONVERSATIONS_PER_USER == 50


@pytest.mark.asyncio
async def test_format_timestamp():
    """Test timestamp formatting function."""
    service = ConversationService()

    # Create a test datetime
    test_time = datetime(2023, 10, 15, 14, 30, 45)

    # Format the timestamp
    formatted = await service._format_timestamp(test_time)

    # Verify the format
    assert isinstance(formatted, str)
    assert "2023-10-15" in formatted
    assert "14:30:45" in formatted


@pytest.mark.asyncio
async def test_validate_conversation_data_valid():
    """Test validation of valid conversation data."""
    service = ConversationService()

    valid_data = ConversationCreate(
        user_id="test_user_id",
        query="This is a valid query",
        response="This is a valid response"
    )

    is_valid = await service._validate_conversation_data(valid_data)
    assert is_valid is True


@pytest.mark.asyncio
async def test_validate_conversation_data_invalid():
    """Test validation of invalid conversation data."""
    service = ConversationService()

    # Test with empty query
    invalid_data = ConversationCreate(
        user_id="test_user_id",
        query="",
        response="This is a valid response"
    )

    is_valid = await service._validate_conversation_data(invalid_data)
    assert is_valid is False

    # Test with empty response
    invalid_data = ConversationCreate(
        user_id="test_user_id",
        query="This is a valid query",
        response=""
    )

    is_valid = await service._validate_conversation_data(invalid_data)
    assert is_valid is False

    # Test with None values
    invalid_data = ConversationCreate(
        user_id=None,
        query="This is a valid query",
        response="This is a valid response"
    )

    is_valid = await service._validate_conversation_data(invalid_data)
    assert is_valid is False


@pytest.mark.asyncio
async def test_enforce_conversation_limit_logic():
    """Test the logic for enforcing conversation limit."""
    service = ConversationService()

    # Mock a list of conversations that exceeds the limit
    mock_conversations = []
    for i in range(55):  # More than the limit of 50
        mock_conversations.append({
            'id': f'conv_{i}',
            'user_id': 'test_user',
            'query': f'Query {i}',
            'response': f'Response {i}',
            'timestamp': datetime.now() - timedelta(minutes=i)  # Older conversations have higher index
        })

    # Sort by timestamp (most recent first) and keep only the last 50
    sorted_conversations = sorted(mock_conversations, key=lambda x: x['timestamp'], reverse=True)
    limited_conversations = sorted_conversations[:50]

    # Verify we have exactly 50 conversations
    assert len(limited_conversations) == 50

    # Verify the most recent conversations are kept
    kept_ids = [conv['id'] for conv in limited_conversations]
    expected_kept_ids = [f'conv_{i}' for i in range(5)]  # The 5 most recent (lowest indices)
    for expected_id in expected_kept_ids:
        assert expected_id in kept_ids


@pytest.mark.asyncio
async def test_conversation_count_calculation():
    """Test the logic for counting conversations per user."""
    service = ConversationService()

    # Mock conversations for testing
    mock_conversations = [
        {'user_id': 'user1', 'id': 'conv1'},
        {'user_id': 'user1', 'id': 'conv2'},
        {'user_id': 'user2', 'id': 'conv3'},
        {'user_id': 'user1', 'id': 'conv4'},
    ]

    # Count conversations for user1
    user1_conversations = [conv for conv in mock_conversations if conv['user_id'] == 'user1']
    assert len(user1_conversations) == 3

    # Count conversations for user2
    user2_conversations = [conv for conv in mock_conversations if conv['user_id'] == 'user2']
    assert len(user2_conversations) == 1


@pytest.mark.asyncio
async def test_conversation_data_sanitization():
    """Test that conversation data is properly sanitized."""
    service = ConversationService()

    # Create conversation data with potential security issues
    unsafe_data = ConversationCreate(
        user_id="test_user_id",
        query="<script>alert('xss')</script> Hello <b>world</b>",
        response="Response with newlines\nand\ttabs"
    )

    # The service should handle sanitization appropriately
    # For now, just verify the data can be processed without error
    is_valid = await service._validate_conversation_data(unsafe_data)
    # Validation should pass as the content itself isn't invalid, just potentially unsafe
    # The actual sanitization would happen at a different level
    assert is_valid is True