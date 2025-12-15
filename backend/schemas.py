from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime
from enum import Enum


class ResponseStatus(str, Enum):
    SUCCESS = "success"
    ERROR = "error"
    PARTIAL = "partial"


class Message(BaseModel):
    """Represents a chat message sent from the UI to the agent"""
    content: str = Field(..., min_length=1, max_length=4000, description="The actual message content from the user")
    user_id: Optional[str] = Field(default=None, pattern=r"^[a-zA-Z0-9-_]+$", description="Identifier for the user sending the message")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the message was sent")
    metadata: Optional[Dict[str, Any]] = Field(default=None, description="Additional context or parameters for the agent")

    class Config:
        json_schema_extra = {
            "example": {
                "content": "Hello, how are you?",
                "user_id": "user-123",
                "timestamp": "2025-12-13T10:00:00Z",
                "metadata": {"context": "full-book"}
            }
        }


class Response(BaseModel):
    """Represents the agent's response to a user message"""
    content: str = Field(..., description="The agent's response content")
    message_id: str = Field(..., description="Reference to the original message")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the response was generated")
    status: ResponseStatus = Field(..., description="Status of the response (success, error, partial)")

    class Config:
        json_schema_extra = {
            "example": {
                "content": "I'm doing well, thank you for asking!",
                "message_id": "msg-456",
                "timestamp": "2025-12-13T10:00:01Z",
                "status": "success"
            }
        }


class ChatRequest(BaseModel):
    """API request payload for the chat endpoint"""
    message: Message = Field(..., description="The message object to send to the agent")
    stream: Optional[bool] = Field(default=False, description="Whether to use streaming response (default: false)")

    class Config:
        json_schema_extra = {
            "example": {
                "message": {
                    "content": "Hello, how are you?",
                    "user_id": "user-123",
                    "timestamp": "2025-12-13T10:00:00Z",
                    "metadata": {"context": "full-book"}
                },
                "stream": False
            }
        }


class ChatResponse(BaseModel):
    """API response for the chat endpoint"""
    response: Response = Field(..., description="The agent's response")
    status_code: int = Field(default=200, description="HTTP status code")
    error: Optional[Dict[str, Any]] = Field(default=None, description="Error details if request failed")

    class Config:
        json_schema_extra = {
            "example": {
                "response": {
                    "content": "I'm doing well, thank you for asking!",
                    "message_id": "msg-456",
                    "timestamp": "2025-12-13T10:00:01Z",
                    "status": "success"
                },
                "status_code": 200
            }
        }


class ErrorResponse(BaseModel):
    """Error response for the API"""
    error: str = Field(..., description="Error code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[Dict[str, Any]] = Field(default=None, description="Additional error details")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "validation_error",
                "message": "Invalid input provided",
                "details": {"field": "content", "reason": "Content cannot be empty"}
            }
        }


class InteractionLog(BaseModel):
    """Represents logged interaction metadata for analytics in Neon database"""
    id: Optional[str] = Field(default=None, description="Unique identifier for the log entry")
    user_id: Optional[str] = Field(default=None, description="User identifier if available")
    session_id: Optional[str] = Field(default=None, description="Session identifier for grouping related interactions")
    request_content: str = Field(..., description="The original user message content")
    response_content: str = Field(default="", description="The agent's response content")
    request_timestamp: datetime = Field(default_factory=datetime.now, description="When the request was received")
    response_timestamp: Optional[datetime] = Field(default=None, description="When the response was completed")
    response_time_ms: Optional[int] = Field(default=None, ge=0, description="Time taken for the agent to respond")
    agent_model: Optional[str] = Field(default=None, description="The model used by the agent")
    error_occurred: bool = Field(default=False, description="Whether an error occurred during processing")
    error_message: Optional[str] = Field(default=None, description="Error message if an error occurred")
    metadata: Optional[Dict[str, Any]] = Field(default=None, description="Additional metadata for analytics")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "log-123",
                "user_id": "user-123",
                "session_id": "session-456",
                "request_content": "Hello, how are you?",
                "response_content": "I'm doing well, thank you!",
                "request_timestamp": "2025-12-13T10:00:00Z",
                "response_timestamp": "2025-12-13T10:00:01Z",
                "response_time_ms": 1000,
                "agent_model": "gpt-4",
                "error_occurred": False,
                "metadata": {"context": "full-book"}
            }
        }