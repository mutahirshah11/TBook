"""
Conversation model for representing user conversation history.
"""
from pydantic import BaseModel, validator
from typing import Optional
from datetime import datetime
import uuid


class Conversation(BaseModel):
    """
    Conversation model representing a single conversation entry in user history.
    """
    id: Optional[str] = None
    user_id: str
    query: str
    response: str
    timestamp: Optional[datetime] = None
    created_at: Optional[datetime] = None

    class Config:
        # Allow ORM mode for database integration
        from_attributes = True

    def __init__(self, **data):
        super().__init__(**data)
        # Generate ID if not provided
        if not self.id:
            self.id = str(uuid.uuid4())
        # Set timestamps if not provided
        if not self.timestamp:
            self.timestamp = datetime.utcnow()
        if not self.created_at:
            self.created_at = datetime.utcnow()


class ConversationCreate(BaseModel):
    """Model for creating a new conversation."""
    user_id: str
    query: str
    response: str

    @validator('query', 'response')
    def validate_content(cls, v):
        """Validate that query and response are not empty."""
        if not v or not v.strip():
            raise ValueError('Query and response cannot be empty')
        return v.strip()

    @validator('user_id')
    def validate_user_id(cls, v):
        """Validate user ID."""
        if not v or not v.strip():
            raise ValueError('User ID is required')
        return v.strip()


class ConversationUpdate(BaseModel):
    """Model for updating conversation information."""
    query: Optional[str] = None
    response: Optional[str] = None


class ConversationInDB(Conversation):
    """Conversation model as stored in the database."""
    pass


class ConversationPublic(BaseModel):
    """Public representation of conversation."""
    id: str
    user_id: str
    query: str
    response: str
    timestamp: datetime

    class Config:
        from_attributes = True