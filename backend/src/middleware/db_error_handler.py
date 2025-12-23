"""
Database error handling middleware for FastAPI.
Handles database-related errors gracefully and provides appropriate responses.
"""
from typing import Callable, Awaitable
from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
import logging
from utils.database import neon_db
from src.services.fallback_service import fallback_service


class DatabaseErrorHandlerMiddleware(BaseHTTPMiddleware):
    """
    Middleware to handle database errors gracefully.
    Intercepts requests and responses to manage database availability.
    """

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        # Check database health at the beginning of the request if needed
        if self.is_database_dependent_endpoint(request):
            # Check if database is available
            if not await self.is_database_available():
                # Set a flag in the request state to indicate DB unavailability
                request.state.db_unavailable = True
                logging.warning(f"Database unavailable for request: {request.method} {request.url}")
            else:
                request.state.db_unavailable = False

        try:
            # Process the request
            response = await call_next(request)

            # Check if this was a database error that we handled
            if hasattr(request.state, 'db_unavailable') and request.state.db_unavailable:
                # Log that we served a request with fallback
                logging.info(f"Served request with fallback: {request.method} {request.url}")

            return response
        except Exception as e:
            # Check if this is a database-related error
            if self.is_database_error(e):
                logging.error(f"Database error in {request.method} {request.url}: {str(e)}")

                # Try to handle with fallback if available
                if await self.handle_with_fallback(request, e):
                    # Return a response indicating graceful degradation
                    return JSONResponse(
                        status_code=status.HTTP_200_OK,
                        content={
                            "message": "Service operating in degraded mode",
                            "error": "Database temporarily unavailable, using fallback",
                            "fallback_active": True
                        }
                    )

                # If fallback isn't available or fails, return appropriate error
                return JSONResponse(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    content={
                        "detail": "Service temporarily unavailable due to database issues",
                        "error_type": "database_unavailable"
                    }
                )
            else:
                # Re-raise if it's not a database error
                raise e

    def is_database_dependent_endpoint(self, request: Request) -> bool:
        """
        Determine if the current endpoint depends on database availability.
        """
        # List of endpoints that require database access
        db_dependent_paths = [
            "/api/v1/auth",
            "/api/v1/conversations",
            "/api/v1/users"
        ]

        request_path = request.url.path

        # Check if the path starts with any of the DB-dependent prefixes
        for db_path in db_dependent_paths:
            if request_path.startswith(db_path):
                return True

        return False

    async def is_database_available(self) -> bool:
        """
        Check if the database is available.
        """
        try:
            if not neon_db.pool:
                return False

            # Try a simple query to test connectivity
            async with neon_db.pool.acquire() as conn:
                await conn.fetchval("SELECT 1")
                return True
        except Exception:
            return False

    def is_database_error(self, error: Exception) -> bool:
        """
        Determine if the error is database-related.
        """
        error_str = str(error).lower()

        # Common database error indicators
        db_error_indicators = [
            "connection", "database", "db", "pool", "server closed",
            "connection refused", "timeout", "connection lost",
            "asyncpg", "postgres", "neon"
        ]

        return any(indicator in error_str for indicator in db_error_indicators)

    async def handle_with_fallback(self, request: Request, error: Exception) -> bool:
        """
        Attempt to handle the request using fallback mechanisms.
        """
        # For now, we'll just enable fallback mode
        # In a real implementation, this would involve more sophisticated fallback logic
        fallback_service.set_fallback_mode(True)
        return True


def setup_db_error_middleware(app):
    """
    Set up the database error handling middleware for the FastAPI app.
    """
    app.add_middleware(DatabaseErrorHandlerMiddleware)
    logging.info("Database error handling middleware registered")