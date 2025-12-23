"""
User model for representing user accounts in the system.
Prepared for future Better Auth integration.
"""
from pydantic import BaseModel, EmailStr, validator
from typing import Optional
from datetime import datetime
import uuid


class User(BaseModel):
    """
    User model representing a registered user account.
    Prepared for future Better Auth integration.
    """
    id: Optional[str] = None
    email: EmailStr
    password_hash: str
    created_at: Optional[datetime] = None

    class Config:
        # Allow ORM mode for database integration
        from_attributes = True

    @validator('email')
    def validate_email_format(cls, v):
        """Validate email format."""
        if not v:
            raise ValueError('Email is required')
        return v.lower().strip()

    def __init__(self, **data):
        super().__init__(**data)
        # Generate ID if not provided
        if not self.id:
            self.id = str(uuid.uuid4())
        # Set creation timestamp if not provided
        if not self.created_at:
            self.created_at = datetime.utcnow()


class UserCreate(BaseModel):
    """Model for creating a new user."""
    email: EmailStr
    password: str

    @validator('password')
    def validate_password(cls, v):
        """Validate password strength."""
        if not v:
            raise ValueError('Password is required')
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters long')
        return v


class UserUpdate(BaseModel):
    """Model for updating user information."""
    email: Optional[EmailStr] = None


class UserInDB(User):
    """User model as stored in the database."""
    pass


class UserPublic(BaseModel):
    """Public representation of user (without sensitive data)."""
    id: str
    email: EmailStr
    created_at: datetime

    class Config:
        from_attributes = True