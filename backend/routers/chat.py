from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, Optional
import time
from datetime import datetime
import uuid
import logging
import html
import re

from pydantic import BaseModel

import sys
import os
# Add the backend directory to the path to allow imports
backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, backend_dir)

from schemas import ChatRequest, ChatResponse, ErrorResponse, ResponseStatus, Message, InteractionLog
from agent_client import agent_client
from utils.database import neon_db
from src.models.conversation import ConversationCreate
from src.services.conversation_service import ConversationService

# Security functions for input sanitization
def sanitize_input(text: str) -> str:
    """
    Sanitize input text to prevent XSS and other injection attacks
    """
    if not text:
        return text

    # Remove potentially dangerous characters/sequences
    # HTML encode to prevent XSS
    sanitized = html.escape(text)

    # Remove control characters that could be used maliciously
    sanitized = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', '', sanitized)

    return sanitized


def validate_user_input(chat_request: ChatRequest) -> ChatRequest:
    """
    Validate and sanitize user input before processing
    """
    # Sanitize the message content
    if chat_request.message.content:
        chat_request.message.content = sanitize_input(chat_request.message.content)

    # Sanitize user_id if present
    if chat_request.message.user_id:
        # Only allow alphanumeric, hyphens, and underscores
        sanitized_user_id = re.sub(r'[^a-zA-Z0-9\-_]', '', chat_request.message.user_id)
        chat_request.message.user_id = sanitized_user_id

    # Sanitize metadata if present
    if chat_request.message.metadata:
        # For metadata, we'll sanitize string values
        if isinstance(chat_request.message.metadata, dict):
            for key, value in chat_request.message.metadata.items():
                if isinstance(value, str):
                    chat_request.message.metadata[key] = sanitize_input(value)

    return chat_request


# Create router
router = APIRouter(
    prefix="/chat",
    tags=["chat"]
)

# Define a schema for the frontend's expected format
class FrontendChatRequest(BaseModel):
    query_text: str
    selected_text: Optional[str] = None
    context_mode: Optional[str] = "full-book"  # "full-book" or "selected-text"

@router.post("/", response_model=ChatResponse)
async def chat_endpoint(chat_request: dict):
    """
    Handle chat requests by forwarding them to the RAG agent and returning the response.
    Also logs the interaction to the Neon database.
    Supports both the original schema format and the frontend's expected format.
    """
    start_time = time.time()
    internal_message = None  # Initialize to ensure it's always defined
    user_id = 'unknown'  # Initialize user_id to ensure it's always defined

    # Check if this is the frontend's format (dict with query_text) or the original format
    if isinstance(chat_request, dict):
        if 'query_text' in chat_request:
            # This is the frontend format - convert to our internal format
            frontend_request = FrontendChatRequest(**chat_request)

            # Create internal message format
            message_content = frontend_request.query_text
            if frontend_request.selected_text:
                message_content += f"\n\nSelected text context: {frontend_request.selected_text}"

            internal_message = Message(
                content=message_content,
                metadata={
                    "context_mode": frontend_request.context_mode,
                    "selected_text": frontend_request.selected_text
                }
            )
        else:
            # This is the original format wrapped in a dict
            chat_request_obj = ChatRequest(**chat_request)
            chat_request_obj = validate_user_input(chat_request_obj)
            internal_message = chat_request_obj.message
    else:
        # This is already a ChatRequest object (original format)
        chat_request = validate_user_input(chat_request)
        internal_message = chat_request.message

    # Set user_id after internal_message is created
    user_id = getattr(internal_message, 'user_id', 'unknown')

    try:
        # Log the incoming request
        logging.info(f"Received chat request for user: {user_id}")

        # Call the agent
        agent_response = await agent_client.run_agent(internal_message)

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Create the response
        response = ChatResponse(
            response=agent_response,
            status_code=200
        )

        # Log interaction to Neon database
        interaction_log = InteractionLog(
            id=str(uuid.uuid4()),
            user_id=user_id,
            request_content=internal_message.content,
            response_content=agent_response.content,
            request_timestamp=internal_message.timestamp,
            response_timestamp=datetime.now(),
            response_time_ms=response_time_ms,
            error_occurred=(agent_response.status == ResponseStatus.ERROR),
            error_message=None if agent_response.status != ResponseStatus.ERROR else "Agent error",
            metadata=internal_message.metadata
        )

        # Log to database asynchronously (with graceful error handling)
        try:
            await neon_db.log_interaction(interaction_log)
        except Exception as e:
            logging.error(f"Failed to log interaction to database: {str(e)}")
            # Try to store in fallback if database is unavailable
            from src.services.fallback_service import fallback_service
            try:
                await fallback_service.store_conversation_fallback(
                    user_id, internal_message.content, agent_response.content
                )
                logging.info(f"Stored interaction in fallback for user {user_id}")
            except Exception as fallback_error:
                logging.error(f"Failed to store interaction in fallback: {str(fallback_error)}")

        # Save conversation to conversation history (if user_id is available)
        if user_id and user_id != 'unknown':
            try:
                conversation_data = ConversationCreate(
                    user_id=user_id,
                    query=internal_message.content,
                    response=agent_response.content
                )
                conversation_service = ConversationService()
                await conversation_service.save_conversation(conversation_data)
                logging.info(f"Conversation saved to history for user {user_id}")
            except Exception as e:
                logging.error(f"Failed to save conversation to history: {str(e)}")
                # Try to store in fallback if main database is unavailable
                from src.services.fallback_service import fallback_service
                try:
                    await fallback_service.store_conversation_fallback(
                        user_id, internal_message.content, agent_response.content
                    )
                    logging.info(f"Stored conversation in fallback for user {user_id}")
                except Exception as fallback_error:
                    logging.error(f"Failed to store conversation in fallback: {str(fallback_error)}")

        return response

    except Exception as e:
        logging.error(f"Error in chat endpoint: {str(e)}")

        # Log error interaction
        error_interaction = InteractionLog(
            id=str(uuid.uuid4()),
            user_id=user_id,
            request_content=internal_message.content if internal_message else "",
            response_content="",
            request_timestamp=internal_message.timestamp if internal_message else datetime.now(),
            response_timestamp=datetime.now(),
            response_time_ms=int((time.time() - start_time) * 1000),
            error_occurred=True,
            error_message=str(e),
            metadata=internal_message.metadata if internal_message else None
        )

        await neon_db.log_interaction(error_interaction)

        # Raise HTTP exception which will be handled by our error handlers
        raise HTTPException(
            status_code=500,
            detail=f"Error processing chat request: {str(e)}"
        )


@router.post("/stream")
async def chat_stream_endpoint(chat_request: dict):
    """
    Handle streaming chat requests by forwarding them to the RAG agent
    and streaming the response back to the client.
    Also logs the interaction to the Neon database.
    Supports both the original schema format and the frontend's expected format.
    """
    from fastapi.responses import StreamingResponse
    import json
    import time
    import uuid
    from datetime import datetime

    # Initialize variables to ensure they're always defined
    internal_message = None
    user_id = 'unknown'

    # Check if this is the frontend's format (dict with query_text) or the original format
    if isinstance(chat_request, dict):
        if 'query_text' in chat_request:
            # This is the frontend format - convert to our internal format
            frontend_request = FrontendChatRequest(**chat_request)

            # Create internal message format
            message_content = frontend_request.query_text
            if frontend_request.selected_text:
                message_content += f"\n\nSelected text context: {frontend_request.selected_text}"

            internal_message = Message(
                content=message_content,
                metadata={
                    "context_mode": frontend_request.context_mode,
                    "selected_text": frontend_request.selected_text
                }
            )
        else:
            # This is the original format wrapped in a dict
            chat_request_obj = ChatRequest(**chat_request)
            chat_request_obj = validate_user_input(chat_request_obj)
            internal_message = chat_request_obj.message
    else:
        # This is already a ChatRequest object (original format)
        chat_request = validate_user_input(chat_request)
        internal_message = chat_request.message

    # Set user_id after internal_message is created
    user_id = getattr(internal_message, 'user_id', 'unknown')

    # Generate a unique ID for this interaction to use for logging
    interaction_id = str(uuid.uuid4())
    start_time = time.time()

    async def log_stream_completion(user_id: str, request_content: str, message_timestamp: datetime, metadata: dict = None):
        """
        Background task to log the completion of the streaming interaction
        In a real implementation, we would need to capture the full response content
        and calculate the response time.
        For now, we'll log a placeholder completion record.
        """
        # In a real implementation, we would have access to the full response content
        # and the actual response time. Since we can't capture that from the streaming
        # response directly, we'll log a completion record with basic information.
        completion_log = InteractionLog(
            id=interaction_id,
            user_id=user_id,
            request_content=request_content,
            response_content="Streaming response completed",  # Placeholder
            request_timestamp=message_timestamp,
            response_timestamp=datetime.now(),
            response_time_ms=int((time.time() - start_time) * 1000),  # Approximate time
            agent_model=None,
            error_occurred=False,
            error_message=None,
            metadata=metadata
        )

        # Log to database asynchronously (with graceful error handling)
        try:
            await neon_db.log_interaction(completion_log)
        except Exception as e:
            logging.error(f"Failed to log streaming interaction to database: {str(e)}")
            # Try to store in fallback if database is unavailable
            from src.services.fallback_service import fallback_service
            try:
                await fallback_service.store_conversation_fallback(
                    user_id, request_content, "Streaming response completed"
                )
                logging.info(f"Stored streaming interaction in fallback for user {user_id}")
            except Exception as fallback_error:
                logging.error(f"Failed to store streaming interaction in fallback: {str(fallback_error)}")

        # Save conversation to conversation history (if user_id is available)
        if user_id and user_id != 'unknown':
            try:
                conversation_data = ConversationCreate(
                    user_id=user_id,
                    query=request_content,
                    response="Streaming response completed"  # Placeholder for now
                )
                conversation_service = ConversationService()
                await conversation_service.save_conversation(conversation_data)
                logging.info(f"Streaming conversation saved to history for user {user_id}")
            except Exception as e:
                logging.error(f"Failed to save streaming conversation to history: {str(e)}")
                # Try to store in fallback if main database is unavailable
                from src.services.fallback_service import fallback_service
                try:
                    await fallback_service.store_conversation_fallback(
                        user_id, request_content, "Streaming response completed"
                    )
                    logging.info(f"Stored streaming conversation in fallback for user {user_id}")
                except Exception as fallback_error:
                    logging.error(f"Failed to store streaming conversation in fallback: {str(fallback_error)}")

    async def event_generator():
        try:
            # Log the incoming request
            user_id = getattr(internal_message, 'user_id', 'unknown')
            logging.info(f"Received streaming chat request for user: {user_id}")

            # Call the agent streaming function with logging
            async for chunk in agent_client.run_agent_stream_with_logging(internal_message, interaction_id):
                # Yield Server-Sent Events
                yield f"data: {json.dumps(chunk)}\n\n"

        except Exception as e:
            logging.error(f"Error in chat stream endpoint: {str(e)}")

            # Log error interaction immediately
            error_interaction = InteractionLog(
                id=interaction_id,  # Use the same interaction ID for consistency
                user_id=user_id,
                request_content=internal_message.content if internal_message else "",
                response_content="",  # Content will be from the logged chunks if any were processed
                request_timestamp=internal_message.timestamp if internal_message else datetime.now(),
                response_timestamp=datetime.now(),
                response_time_ms=int((time.time() - start_time) * 1000),
                error_occurred=True,
                error_message=str(e),
                metadata=internal_message.metadata if internal_message else None
            )

            await neon_db.log_interaction(error_interaction)

            yield f"data: {json.dumps({'error': str(e), 'status': 'error'})}\n\n"

    # Log the start of the streaming interaction
    stream_start_log = InteractionLog(
        id=interaction_id,
        user_id=user_id,
        request_content=internal_message.content if internal_message else "",
        response_content="",  # Will be updated when stream completes
        request_timestamp=internal_message.timestamp if internal_message else datetime.now(),
        response_timestamp=start_time,
        response_time_ms=None,  # Will be calculated when stream completes
        agent_model=None,
        error_occurred=False,
        error_message=None,
        metadata=internal_message.metadata if internal_message else None
    )

    await neon_db.log_interaction(stream_start_log)

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream"
    )