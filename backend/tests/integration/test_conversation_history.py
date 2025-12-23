"""
Integration tests for conversation history functionality.
Tests the complete conversation storage and retrieval flow.
"""
import pytest
import asyncio
from datetime import datetime
from src.services.conversation_service import ConversationService
from src.models.conversation import ConversationCreate
from src.services.user_service import UserService
import uuid


@pytest.mark.asyncio
async def test_conversation_storage_and_retrieval():
    """Test the complete conversation storage and retrieval flow."""
    # Create a test user first
    user_service = UserService()
    test_email = f"conv_test_{uuid.uuid4()}@example.com"
    test_password = "securepassword123"

    user_id = await user_service.register_user(test_email, test_password)
    assert user_id is not None

    # Create conversation service
    conversation_service = ConversationService()

    # Create a conversation
    conversation_data = ConversationCreate(
        user_id=user_id,
        query="Hello, how are you?",
        response="I'm doing well, thank you for asking!"
    )

    conversation_id = await conversation_service.save_conversation(conversation_data)
    assert conversation_id is not None

    # Retrieve conversations for the user
    conversations = await conversation_service.get_user_conversations(user_id)
    assert len(conversations) == 1
    assert conversations[0]['user_id'] == user_id
    assert conversations[0]['query'] == conversation_data.query
    assert conversations[0]['response'] == conversation_data.response


@pytest.mark.asyncio
async def test_multiple_conversations_storage():
    """Test storing multiple conversations for a user."""
    # Create a test user
    user_service = UserService()
    test_email = f"multi_conv_test_{uuid.uuid4()}@example.com"
    test_password = "securepassword123"

    user_id = await user_service.register_user(test_email, test_password)
    assert user_id is not None

    # Create conversation service
    conversation_service = ConversationService()

    # Create multiple conversations
    conversations_data = [
        ConversationCreate(user_id=user_id, query="First query", response="First response"),
        ConversationCreate(user_id=user_id, query="Second query", response="Second response"),
        ConversationCreate(user_id=user_id, query="Third query", response="Third response")
    ]

    # Save all conversations
    conversation_ids = []
    for conv_data in conversations_data:
        conv_id = await conversation_service.save_conversation(conv_data)
        conversation_ids.append(conv_id)
        assert conv_id is not None

    # Retrieve conversations for the user
    conversations = await conversation_service.get_user_conversations(user_id)
    assert len(conversations) == 3

    # Verify all conversations were saved
    saved_queries = [conv['query'] for conv in conversations]
    expected_queries = [conv.query for conv in conversations_data]
    assert set(saved_queries) == set(expected_queries)


@pytest.mark.asyncio
async def test_conversation_limit_enforcement():
    """Test that only the last 50 conversations are kept per user."""
    # Create a test user
    user_service = UserService()
    test_email = f"limit_test_{uuid.uuid4()}@example.com"
    test_password = "securepassword123"

    user_id = await user_service.register_user(test_email, test_password)
    assert user_id is not None

    # Create conversation service
    conversation_service = ConversationService()

    # Create 55 conversations (exceeding the limit)
    for i in range(55):
        conversation_data = ConversationCreate(
            user_id=user_id,
            query=f"Query {i}",
            response=f"Response {i}"
        )
        conv_id = await conversation_service.save_conversation(conversation_data)
        assert conv_id is not None

    # Retrieve conversations for the user
    conversations = await conversation_service.get_user_conversations(user_id)

    # Should only have the last 50 conversations
    assert len(conversations) == 50

    # Verify that the most recent conversations are kept
    saved_queries = [conv['query'] for conv in conversations]
    # The saved queries should be from the last 50 (queries 5-54 if created in order 0-54)
    expected_queries = [f"Query {i}" for i in range(5, 55)]
    assert set(saved_queries) == set(expected_queries)


@pytest.mark.asyncio
async def test_conversation_retrieval_by_different_users():
    """Test that users only see their own conversations."""
    # Create two test users
    user_service = UserService()

    test_email1 = f"user1_{uuid.uuid4()}@example.com"
    test_email2 = f"user2_{uuid.uuid4()}@example.com"
    test_password = "securepassword123"

    user_id1 = await user_service.register_user(test_email1, test_password)
    user_id2 = await user_service.register_user(test_email2, test_password)
    assert user_id1 is not None
    assert user_id2 is not None
    assert user_id1 != user_id2

    # Create conversation service
    conversation_service = ConversationService()

    # Create conversations for user 1
    conv1_data = ConversationCreate(user_id=user_id1, query="User1 query", response="User1 response")
    conv1_id = await conversation_service.save_conversation(conv1_data)
    assert conv1_id is not None

    # Create conversations for user 2
    conv2_data = ConversationCreate(user_id=user_id2, query="User2 query", response="User2 response")
    conv2_id = await conversation_service.save_conversation(conv2_data)
    assert conv2_id is not None

    # Each user should only see their own conversations
    user1_conversations = await conversation_service.get_user_conversations(user_id1)
    user2_conversations = await conversation_service.get_user_conversations(user_id2)

    assert len(user1_conversations) == 1
    assert len(user2_conversations) == 1
    assert user1_conversations[0]['query'] == "User1 query"
    assert user2_conversations[0]['query'] == "User2 query"


@pytest.mark.asyncio
async def test_conversation_with_invalid_user():
    """Test conversation creation with invalid user ID."""
    # Create conversation service
    conversation_service = ConversationService()

    # Try to create conversation with invalid user ID
    invalid_user_id = "invalid_user_id"
    conversation_data = ConversationCreate(
        user_id=invalid_user_id,
        query="Test query",
        response="Test response"
    )

    conversation_id = await conversation_service.save_conversation(conversation_data)
    # This might succeed or fail depending on foreign key constraints, but shouldn't crash
    # If foreign key constraint is enforced, this should fail; otherwise it might succeed
    # We'll just verify it doesn't crash the system