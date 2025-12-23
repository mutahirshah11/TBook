"""
Authentication utilities for password hashing and verification.
Uses bcrypt for secure password hashing.
"""
import bcrypt
from typing import Optional


def hash_password(password: str) -> str:
    """
    Hash a password using bcrypt.

    Args:
        password: The plain text password to hash

    Returns:
        The hashed password as a string
    """
    if not password:
        raise ValueError("Password cannot be empty")

    # Generate a salt and hash the password
    salt = bcrypt.gensalt()
    hashed = bcrypt.hashpw(password.encode('utf-8'), salt)

    # Return as string (bcrypt.hashpw returns bytes)
    return hashed.decode('utf-8')


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password.

    Args:
        plain_password: The plain text password to verify
        hashed_password: The hashed password to compare against

    Returns:
        True if the password matches, False otherwise
    """
    if not plain_password or not hashed_password:
        return False

    try:
        # Verify the password
        return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))
    except Exception:
        # If there's an error in verification, return False
        return False


def is_valid_email(email: str) -> bool:
    """
    Basic email validation.

    Args:
        email: The email address to validate

    Returns:
        True if the email is valid, False otherwise
    """
    if not email or '@' not in email:
        return False

    # Basic check: must have @ and at least one dot after @
    parts = email.split('@')
    if len(parts) != 2:
        return False

    local, domain = parts
    if not local or not domain or '.' not in domain:
        return False

    return True