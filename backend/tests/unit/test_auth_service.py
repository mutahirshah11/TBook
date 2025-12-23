"""
Unit tests for authentication service functionality.
Tests password hashing and verification functions in isolation.
"""
import pytest
import asyncio
from utils.auth import hash_password, verify_password


def test_password_hashing():
    """Test that password hashing works correctly."""
    password = "securepassword123"

    # Hash the password
    hashed = hash_password(password)

    # Verify it's not the same as original
    assert hashed != password

    # Verify it has expected length/format (bcrypt hash)
    assert len(hashed) > 0
    assert hashed.startswith('$2b$') or hashed.startswith('$2a$') or hashed.startswith('$2y$')


def test_password_verification_success():
    """Test that password verification works for correct password."""
    password = "securepassword123"

    # Hash the password
    hashed = hash_password(password)

    # Verify the same password
    is_valid = verify_password(password, hashed)

    assert is_valid is True


def test_password_verification_failure():
    """Test that password verification fails for wrong password."""
    password = "securepassword123"
    wrong_password = "wrongpassword456"

    # Hash the original password
    hashed = hash_password(password)

    # Verify with wrong password
    is_valid = verify_password(wrong_password, hashed)

    assert is_valid is False


def test_password_verification_with_empty_values():
    """Test password verification with empty values."""
    # Test with empty password
    hashed = hash_password("somepassword")
    is_valid = verify_password("", hashed)
    assert is_valid is False

    # Test with empty hash
    is_valid = verify_password("somepassword", "")
    assert is_valid is False

    # Test with both empty
    is_valid = verify_password("", "")
    assert is_valid is False


def test_different_passwords_produce_different_hashes():
    """Test that different passwords produce different hashes."""
    password1 = "password1"
    password2 = "password2"

    hash1 = hash_password(password1)
    hash2 = hash_password(password2)

    # Even with the same password, bcrypt should produce different salts
    # But for different passwords, hashes should definitely be different
    assert hash1 != hash2

    # Verify each password only works with its own hash
    assert verify_password(password1, hash1) is True
    assert verify_password(password2, hash2) is True
    assert verify_password(password1, hash2) is False
    assert verify_password(password2, hash1) is False