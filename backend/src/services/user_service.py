"""
User service for handling user authentication and management operations.
Foundation for future Better Auth integration.
"""
from typing import Optional, Dict, Any
from src.models.user import User, UserCreate, UserPublic
from utils.auth import hash_password, verify_password
from utils.database import neon_db
from src.services.database_service import DatabaseService
import logging
import uuid
from datetime import datetime


class UserService:
    """Service class for user authentication and management operations."""

    def __init__(self):
        self.db_service = DatabaseService()

    async def register_user(self, email: str, password: str) -> Optional[str]:
        """
        Register a new user with email and password.
        Returns the user ID if successful, None if failed (e.g., duplicate email).
        """
        try:
            # Hash the password
            password_hash = hash_password(password)

            # Generate user ID
            user_id = str(uuid.uuid4())

            # Insert user into database
            query = """
                INSERT INTO users (id, email, password_hash, created_at)
                VALUES ($1, $2, $3, $4)
                RETURNING id
            """

            async with neon_db.pool.acquire() as conn:
                result = await conn.fetchval(query, user_id, email.lower().strip(), password_hash, datetime.utcnow())

                if result:
                    logging.info(f"User registered successfully: {email}")
                    return str(result)
                else:
                    logging.warning(f"Failed to register user: {email}")
                    return None
        except Exception as e:
            logging.error(f"Error registering user {email}: {str(e)}")
            # Check if it's a unique constraint violation (duplicate email)
            if "duplicate key value violates unique constraint" in str(e).lower():
                return None
            return None

    async def authenticate_user(self, email: str, password: str) -> Optional[Dict[str, Any]]:
        """
        Authenticate a user with email and password.
        Returns user data if authentication is successful, None otherwise.
        """
        try:
            # Fetch user from database
            query = """
                SELECT id, email, password_hash FROM users WHERE email = $1
            """

            async with neon_db.pool.acquire() as conn:
                user_record = await conn.fetchrow(query, email.lower().strip())

            if not user_record:
                logging.info(f"Authentication failed: user not found - {email}")
                return None

            # Verify password
            stored_hash = user_record['password_hash']
            if verify_password(password, stored_hash):
                logging.info(f"User authenticated successfully: {email}")
                return {
                    'id': str(user_record['id']),
                    'email': user_record['email']
                }
            else:
                logging.info(f"Authentication failed: invalid password - {email}")
                return None
        except Exception as e:
            logging.error(f"Error authenticating user {email}: {str(e)}")
            return None

    async def get_user_by_id(self, user_id: str) -> Optional[UserPublic]:
        """Get user by ID."""
        try:
            query = """
                SELECT id, email, created_at FROM users WHERE id = $1
            """

            async with neon_db.pool.acquire() as conn:
                user_record = await conn.fetchrow(query, user_id)

            if user_record:
                return UserPublic(
                    id=str(user_record['id']),
                    email=user_record['email'],
                    created_at=user_record['created_at']
                )
            return None
        except Exception as e:
            logging.error(f"Error fetching user by ID {user_id}: {str(e)}")
            return None

    async def get_user_by_email(self, email: str) -> Optional[UserPublic]:
        """Get user by email."""
        try:
            query = """
                SELECT id, email, created_at FROM users WHERE email = $1
            """

            async with neon_db.pool.acquire() as conn:
                user_record = await conn.fetchrow(query, email.lower().strip())

            if user_record:
                return UserPublic(
                    id=str(user_record['id']),
                    email=user_record['email'],
                    created_at=user_record['created_at']
                )
            return None
        except Exception as e:
            logging.error(f"Error fetching user by email {email}: {str(e)}")
            return None

    async def update_user(self, user_id: str, email: Optional[str] = None) -> bool:
        """Update user information."""
        try:
            updates = []
            params = [user_id]  # user_id is always the last parameter for WHERE clause
            param_count = 1

            if email:
                updates.append(f"email = ${param_count + 1}")
                params.insert(param_count, email.lower().strip())
                param_count += 1

            if not updates:
                return True  # Nothing to update, but operation is successful

            query = f"""
                UPDATE users SET {', '.join(updates)} WHERE id = ${param_count}
            """

            async with neon_db.pool.acquire() as conn:
                result = await conn.execute(query, *params)

            # Check if any rows were affected
            return "UPDATE 0" not in result or "UPDATE 1" in result
        except Exception as e:
            logging.error(f"Error updating user {user_id}: {str(e)}")
            return False

    async def delete_user(self, user_id: str) -> bool:
        """Delete a user by ID."""
        try:
            query = "DELETE FROM users WHERE id = $1"

            async with neon_db.pool.acquire() as conn:
                result = await conn.execute(query, user_id)

            return "DELETE 0" not in result or "DELETE 1" in result
        except Exception as e:
            logging.error(f"Error deleting user {user_id}: {str(e)}")
            return False