import sys
import os
# Add the backend directory to the path to allow imports
backend_dir = os.path.dirname(os.path.abspath(__file__))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

# Add the project root to the path to access the agent
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import asyncio
import time
from typing import Dict, Any, AsyncGenerator
from schemas import Message, Response, ResponseStatus, InteractionLog
from pydantic_settings import BaseSettings
import logging

# Import the agent interface at module level to avoid function-level imports
try:
    from backend.agents.book_rag_agent.interface import ask_agent
    from backend.agents.book_rag_agent.settings import settings
    AGENT_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import agent: {e}")
    AGENT_AVAILABLE = False
    # Define a mock ask_agent for fallback
    async def ask_agent(query: str, selected_text=None):
        yield f"Mock response to: {query[:50]}..."

# Import other necessary modules at the module level
from utils.database import neon_db


class AgentClient:
    """
    Thin wrapper around existing agent functions to handle the interface
    between the FastAPI endpoints and the agent without modifying the
    existing agent code.

    This is a placeholder implementation that simulates interaction with
    the existing RAG agent. In a real implementation, this would call
    the actual agent functions like run_agent and run_agent_stream.
    """

    def __init__(self):
        # Load timeout from environment variable
        self.timeout = int(os.getenv("AGENT_TIMEOUT", "30"))
        logging.info("AgentClient initialized with timeout: %d seconds", self.timeout)

    async def run_agent(self, message: Message) -> Response:
        """
        Forward the message to the existing agent and return the response
        This is a placeholder that simulates the agent's response
        """
        try:
            # Implement timeout for agent processing
            response_content = await asyncio.wait_for(
                self._run_agent_internal(message),
                timeout=self.timeout
            )

            return Response(
                content=response_content,
                message_id=message.user_id or "unknown",
                timestamp=message.timestamp,
                status=ResponseStatus.SUCCESS
            )
        except asyncio.TimeoutError:
            logging.error(f"Agent timeout after {self.timeout} seconds for message: {message.content[:50]}...")
            return Response(
                content=f"Request timed out after {self.timeout} seconds",
                message_id=message.user_id or "unknown",
                timestamp=message.timestamp,
                status=ResponseStatus.ERROR
            )
        except Exception as e:
            logging.error(f"Error in run_agent: {str(e)}")
            return Response(
                content="An error occurred while processing your request",
                message_id=message.user_id or "unknown",
                timestamp=message.timestamp,
                status=ResponseStatus.ERROR
            )

    async def _run_agent_internal(self, message: Message) -> str:
        """
        Internal method to call the actual RAG agent
        """
        try:
            # Set up the OpenAI client with Gemini for compatibility
            # This configures the global OpenAI client to work with Gemini
            import openai
            openai.api_key = os.getenv("GEMINI_API_KEY")
            openai.base_url = "https://generativelanguage.googleapis.com/v1beta/openai/"  # Gemini-compatible endpoint

            # Agent interface already imported at module level

            # Extract context mode and selected text from metadata if available
            context_mode = "full-book"
            selected_text = None

            if message.metadata:
                if "context_mode" in message.metadata:
                    context_mode = message.metadata["context_mode"]
                if "selected_text" in message.metadata:
                    selected_text = message.metadata["selected_text"]

            # Call the actual agent
            response_parts = []
            async for chunk in ask_agent(message.content, selected_text):
                response_parts.append(chunk)

            response_content = "".join(response_parts)
            return response_content

        except Exception as e:
            logging.error(f"Error calling actual agent: {str(e)}")
            # Fallback to mock response if actual agent fails
            return f"Error: Could not reach the agent - {str(e)}"

    async def run_agent_stream(self, message: Message) -> AsyncGenerator[Dict[str, Any], None]:
        """
        Stream the agent's response in real-time using the actual RAG agent
        """
        try:
            # Use timeout for the entire streaming operation
            async with asyncio.timeout(self.timeout):
                # Set up the OpenAI client with Gemini for compatibility
                import openai
                openai.api_key = os.getenv("GEMINI_API_KEY", "")
                openai.base_url = "https://generativelanguage.googleapis.com/v1beta/openai/"  # Gemini-compatible endpoint

                # Agent interface already imported at module level

                # Extract context mode and selected text from metadata if available
                selected_text = None
                if message.metadata:
                    if "selected_text" in message.metadata:
                        selected_text = message.metadata["selected_text"]

                # Call the actual agent which returns an async generator
                async for chunk in ask_agent(message.content, selected_text):
                    # Yield each chunk from the actual agent
                    yield {
                        "content": chunk,
                        "status": ResponseStatus.SUCCESS.value
                    }

        except asyncio.TimeoutError:
            logging.error(f"Agent stream timeout after {self.timeout} seconds for message: {message.content[:50]}...")
            yield {
                "content": f"Request timed out after {self.timeout} seconds",
                "status": ResponseStatus.ERROR.value
            }
        except Exception as e:
            logging.error(f"Error in run_agent_stream: {str(e)}")
            yield {
                "content": f"An error occurred while processing your request: {str(e)}",
                "status": ResponseStatus.ERROR.value
            }

    async def run_agent_stream_with_logging(self, message: Message, interaction_id: str = None) -> AsyncGenerator[Dict[str, Any], None]:
        """
        Stream the agent's response with logging support
        This version allows for logging the complete interaction when streaming is done
        """
        start_time = time.time()
        full_content = ""  # Initialize here to ensure it's available in exception handlers
        try:
            # Use timeout for the entire streaming operation
            async with asyncio.timeout(self.timeout):
                # Set up the OpenAI client with Gemini for compatibility
                import openai
                openai.api_key = os.getenv("GEMINI_API_KEY", "")
                openai.base_url = "https://generativelanguage.googleapis.com/v1beta/openai/"  # Gemini-compatible endpoint

                # Agent interface already imported at module level

                # Extract context mode and selected text from metadata if available
                selected_text = None
                if message.metadata:
                    if "selected_text" in message.metadata:
                        selected_text = message.metadata["selected_text"]

                # Call the actual agent which returns an async generator
                async for chunk in ask_agent(message.content, selected_text):
                    # Accumulate the full content for logging
                    full_content += chunk

                    # Yield each chunk from the actual agent
                    yield {
                        "content": chunk,
                        "status": ResponseStatus.SUCCESS.value
                    }

                # After streaming completes, log the full interaction to database
                # This would normally be called from the endpoint after the stream completes
                if interaction_id:
                    from datetime import datetime

                    interaction_log = InteractionLog(
                        id=interaction_id,
                        user_id=message.user_id,
                        request_content=message.content,
                        response_content=full_content.strip(),
                        request_timestamp=message.timestamp,
                        response_timestamp=datetime.now(),
                        response_time_ms=int((time.time() - start_time) * 1000),  # Use the global time module imported at the top
                        error_occurred=False,
                        error_message=None,
                        metadata=message.metadata
                    )

                    await neon_db.log_interaction(interaction_log)

        except asyncio.TimeoutError:
            logging.error(f"Agent stream with logging timeout after {self.timeout} seconds for message: {message.content[:50]}...")

            # Log the timeout error to database
            if interaction_id:
                from datetime import datetime

                timeout_interaction = InteractionLog(
                    id=interaction_id,
                    user_id=message.user_id,
                    request_content=message.content,
                    response_content=full_content,  # Include any content that was received before timeout
                    request_timestamp=message.timestamp,
                    response_timestamp=datetime.now(),
                    response_time_ms=int((time.time() - start_time) * 1000),
                    error_occurred=True,
                    error_message=f"Request timed out after {self.timeout} seconds",
                    metadata=message.metadata
                )

                await neon_db.log_interaction(timeout_interaction)

            yield {
                "content": f"Request timed out after {self.timeout} seconds",
                "status": ResponseStatus.ERROR.value
            }
        except Exception as e:
            logging.error(f"Error in run_agent_stream_with_logging: {str(e)}")

            # Log the error to database
            if interaction_id:
                from datetime import datetime

                error_interaction = InteractionLog(
                    id=interaction_id,
                    user_id=message.user_id,
                    request_content=message.content,
                    response_content=full_content,  # Include any content that was received before error
                    request_timestamp=message.timestamp,
                    response_timestamp=datetime.now(),
                    response_time_ms=int((time.time() - start_time) * 1000),
                    error_occurred=True,
                    error_message=str(e),
                    metadata=message.metadata
                )

                await neon_db.log_interaction(error_interaction)

            yield {
                "content": f"An error occurred while processing your request: {str(e)}",
                "status": ResponseStatus.ERROR.value
            }


# Global instance for use in endpoints
agent_client = AgentClient()


# These functions maintain the interface expected by the system
async def run_agent(message: Dict[str, Any]) -> str:
    """
    Wrapper function that maintains compatibility with the expected interface
    """
    msg = Message(**message)
    response = await agent_client.run_agent(msg)
    return response.content


async def run_agent_stream(message: Dict[str, Any]) -> AsyncGenerator[Dict[str, Any], None]:
    """
    Wrapper function that maintains compatibility with the expected streaming interface
    """
    msg = Message(**message)
    async for chunk in agent_client.run_agent_stream(msg):
        yield chunk