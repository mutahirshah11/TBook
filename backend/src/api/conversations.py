"""
Conversation API endpoints for managing conversation history.
"""
from fastapi import APIRouter, HTTPException, status, Query
from typing import Dict, Any, List, Optional
import logging
from src.models.conversation import ConversationCreate, ConversationPublic
from src.services.conversation_service import ConversationService


# Create router
router = APIRouter(prefix="/conversations", tags=["conversations"])


@router.get("/", response_model=List[Dict[str, Any]])
async def get_conversations(
    limit: int = Query(default=50, ge=1, le=50, description="Number of conversations to return (max 50)")
):
    """
    Get user's conversation history.
    Note: This is a placeholder that would integrate with Better Auth in the future.
    For now, this endpoint is not fully functional without proper authentication.
    """
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="This endpoint requires user authentication, which will be implemented with Better Auth"
    )


@router.post("/", status_code=status.HTTP_201_CREATED)
async def create_conversation(conversation_data: ConversationCreate):
    """
    Save a new conversation.
    Note: This is a placeholder that would integrate with Better Auth in the future.
    For now, this endpoint is not fully functional without proper authentication.
    """
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="This endpoint requires user authentication, which will be implemented with Better Auth"
    )


@router.get("/{conversation_id}", response_model=ConversationPublic)
async def get_conversation(conversation_id: str):
    """
    Get a specific conversation by ID.
    Note: This is a placeholder that would integrate with Better Auth in the future.
    For now, this endpoint is not fully functional without proper authentication.
    """
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="This endpoint requires user authentication, which will be implemented with Better Auth"
    )


@router.put("/{conversation_id}")
async def update_conversation(conversation_id: str):
    """
    Update a specific conversation.
    Note: This is a placeholder that would integrate with Better Auth in the future.
    For now, this endpoint is not fully functional without proper authentication.
    """
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="This endpoint requires user authentication, which will be implemented with Better Auth"
    )


@router.delete("/{conversation_id}")
async def delete_conversation(conversation_id: str):
    """
    Delete a specific conversation.
    Note: This is a placeholder that would integrate with Better Auth in the future.
    For now, this endpoint is not fully functional without proper authentication.
    """
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="This endpoint requires user authentication, which will be implemented with Better Auth"
    )


# For integration with existing chat functionality, we'll also add an endpoint
# that can be called internally by the chat service
@router.post("/internal-save", status_code=status.HTTP_201_CREATED)
async def save_conversation_internal(conversation_data: ConversationCreate):
    """
    Internal endpoint to save a conversation without authentication.
    This is intended to be used by the chat service internally.
    """
    # Validate the conversation data
    if not conversation_data.query or not conversation_data.query.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Query is required and cannot be empty"
        )

    if not conversation_data.response or not conversation_data.response.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Response is required and cannot be empty"
        )

    # Attempt to save the conversation
    conversation_service = ConversationService()
    conversation_id = await conversation_service.save_conversation(conversation_data)

    if conversation_id is None:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to save conversation"
        )

    logging.info(f"Conversation saved internally for user {conversation_data.user_id}")
    return {
        "id": conversation_id,
        "user_id": conversation_data.user_id,
        "message": "Conversation saved successfully"
    }