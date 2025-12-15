import pytest
from pydantic import ValidationError
from datetime import datetime
import sys
import os

# Add the backend directory to the path so we can import from it
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from backend.schemas import Message, Response, ChatRequest, ChatResponse, ErrorResponse, InteractionLog, ResponseStatus


def test_message_schema_valid():
    """Test that a valid Message schema is created successfully"""
    message = Message(
        content="Hello, world!",
        user_id="user-123",
        timestamp=datetime.now(),
        metadata={"context": "full-book"}
    )

    assert message.content == "Hello, world!"
    assert message.user_id == "user-123"
    assert message.metadata == {"context": "full-book"}


def test_message_content_validation():
    """Test content length validation"""
    # Test with empty content - should fail
    with pytest.raises(ValidationError):
        Message(content="")

    # Test with content exceeding max length - should fail
    with pytest.raises(ValidationError):
        Message(content="A" * 4001)

    # Test with content at max length - should succeed
    Message(content="A" * 4000)


def test_message_user_id_validation():
    """Test user_id format validation"""
    # Valid user_id formats
    Message(content="test", user_id="valid-user")
    Message(content="test", user_id="valid_user")
    Message(content="test", user_id="valid-user123")

    # Invalid user_id formats - should fail
    with pytest.raises(ValidationError):
        Message(content="test", user_id="invalid user id")
    with pytest.raises(ValidationError):
        Message(content="test", user_id="user@invalid")
    with pytest.raises(ValidationError):
        Message(content="test", user_id="user.invalid")


def test_response_schema():
    """Test Response schema creation"""
    response = Response(
        content="Agent response",
        message_id="msg-123",
        timestamp=datetime.now(),
        status=ResponseStatus.SUCCESS
    )

    assert response.content == "Agent response"
    assert response.message_id == "msg-123"
    assert response.status == ResponseStatus.SUCCESS


def test_chat_request_schema():
    """Test ChatRequest schema creation"""
    message = Message(content="Hello")
    chat_request = ChatRequest(
        message=message,
        stream=True
    )

    assert chat_request.message == message
    assert chat_request.stream is True


def test_chat_response_schema():
    """Test ChatResponse schema creation"""
    response = Response(
        content="Agent response",
        message_id="msg-123",
        timestamp=datetime.now(),
        status=ResponseStatus.SUCCESS
    )

    chat_response = ChatResponse(
        response=response,
        status_code=200
    )

    assert chat_response.response == response
    assert chat_response.status_code == 200


def test_error_response_schema():
    """Test ErrorResponse schema creation"""
    error_response = ErrorResponse(
        error="validation_error",
        message="Invalid input provided",
        details={"field": "content", "reason": "Content cannot be empty"}
    )

    assert error_response.error == "validation_error"
    assert error_response.message == "Invalid input provided"


def test_interaction_log_schema():
    """Test InteractionLog schema creation"""
    interaction_log = InteractionLog(
        id="log-123",
        user_id="user-123",
        session_id="session-456",
        request_content="User message",
        response_content="Agent response",
        request_timestamp=datetime.now(),
        response_timestamp=datetime.now(),
        response_time_ms=1000,
        agent_model="gpt-4",
        error_occurred=False,
        error_message=None,
        metadata={"context": "full-book"}
    )

    assert interaction_log.id == "log-123"
    assert interaction_log.user_id == "user-123"
    assert interaction_log.request_content == "User message"
    assert interaction_log.response_time_ms == 1000


def test_interaction_log_response_time_validation():
    """Test response_time_ms validation"""
    # Should fail with negative value
    with pytest.raises(ValidationError):
        InteractionLog(
            id="log-123",
            user_id="user-123",
            request_content="User message",
            response_content="Agent response",
            request_timestamp=datetime.now(),
            response_time_ms=-100  # Negative value should fail
        )

    # Should succeed with positive value
    InteractionLog(
        id="log-123",
        user_id="user-123",
        request_content="User message",
        response_content="Agent response",
        request_timestamp=datetime.now(),
        response_time_ms=500  # Positive value should succeed
    )


def test_response_status_enum():
    """Test ResponseStatus enum values"""
    assert ResponseStatus.SUCCESS.value == "success"
    assert ResponseStatus.ERROR.value == "error"
    assert ResponseStatus.PARTIAL.value == "partial"

    # Test that these are the only valid values
    with pytest.raises(ValueError):
        ResponseStatus("invalid_status")