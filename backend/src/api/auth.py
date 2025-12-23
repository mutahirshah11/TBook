"""
Authentication API endpoints for user registration and login.
Foundation for future Better Auth integration.
"""
from fastapi import APIRouter, HTTPException, Depends, status
from typing import Dict, Any, Optional
import logging
from src.models.user import UserCreate, UserPublic
from src.services.user_service import UserService
from utils.auth import is_valid_email


# Create router
router = APIRouter(prefix="/auth", tags=["authentication"])


@router.post("/register", response_model=UserPublic, status_code=status.HTTP_201_CREATED)
async def register_user(user_data: UserCreate):
    """
    Register a new user with email and password.
    This endpoint stores user data in the Neon database, preparing for Better Auth integration.
    """
    # Validate email format
    if not is_valid_email(user_data.email):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid email format"
        )

    # Validate password length
    if len(user_data.password) < 8:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Password must be at least 8 characters long"
        )

    # Attempt to register the user
    user_service = UserService()
    user_id = await user_service.register_user(user_data.email, user_data.password)

    if user_id is None:
        # This could be due to duplicate email
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Fetch the created user to return
    created_user = await user_service.get_user_by_id(user_id)
    if not created_user:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Registration succeeded but could not retrieve user"
        )

    logging.info(f"New user registered: {user_data.email}")
    return created_user


@router.post("/login")
async def login_user(credentials: Dict[str, str]):
    """
    Verify user credentials and return user information.
    This endpoint verifies credentials against the Neon database, preparing for Better Auth integration.
    """
    email = credentials.get("email")
    password = credentials.get("password")

    if not email or not password:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email and password are required"
        )

    # Validate email format
    if not is_valid_email(email):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid email format"
        )

    # Attempt to authenticate the user
    user_service = UserService()
    auth_result = await user_service.authenticate_user(email, password)

    if not auth_result:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    logging.info(f"User logged in: {email}")
    return {
        "user_id": auth_result['id'],
        "email": auth_result['email']
    }


@router.get("/me", response_model=UserPublic)
async def get_current_user():
    """
    Get current user information.
    Note: This is a placeholder that would integrate with Better Auth in the future.
    For now, this endpoint is not fully functional without proper authentication middleware.
    """
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="This endpoint requires authentication middleware, which will be implemented with Better Auth"
    )


@router.put("/password")
async def update_password():
    """
    Update user password.
    Note: This is a placeholder that would integrate with Better Auth in the future.
    For now, this endpoint is not fully functional without proper authentication middleware.
    """
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="This endpoint requires authentication middleware, which will be implemented with Better Auth"
    )