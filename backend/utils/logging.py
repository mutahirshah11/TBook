import sys
import os
# Add the backend directory to the path to allow imports
backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, backend_dir)

import logging
import time
from typing import Callable, Any
from fastapi import Request, Response
from fastapi.routing import APIRoute
from starlette.routing import Match
from starlette.types import ASGIApp, Scope, Receive, Send


class LoggingRoute(APIRoute):
    """
    Custom APIRoute that adds logging for all requests
    """
    def get_route_handler(self) -> Callable:
        original_route_handler = super().get_route_handler()

        async def custom_route_handler(request: Request) -> Response:
            # Log request
            start_time = time.time()
            try:
                response = await original_route_handler(request)
            except Exception as e:
                # Log exception
                logging.error(f"Request failed: {request.method} {request.url} - {str(e)}")
                raise
            finally:
                # Calculate duration
                duration = time.time() - start_time

                # Log response
                logging.info(
                    f"{request.method} {request.url.path} "
                    f"Status: {response.status_code} "
                    f"Duration: {duration:.3f}s "
                    f"User-Agent: {request.headers.get('user-agent', 'Unknown')}"
                )
            return response

        return custom_route_handler


def setup_logging_middleware(app):
    """
    Setup logging middleware for the FastAPI app
    """
    # Set up basic logging configuration
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Replace the router with one that uses our custom route class
    app.router.route_class = LoggingRoute

    return app


# Alternative approach: Custom middleware class
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response as StarletteResponse
import json


class LoggingMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        start_time = time.time()

        # Process request
        response = await call_next(request)

        # Calculate duration
        duration = time.time() - start_time

        # Log the request and response
        logging.info(
            f"Request: {request.method} {request.url} | "
            f"Response: {response.status_code} | "
            f"Duration: {duration:.3f}s | "
            f"User-Agent: {request.headers.get('user-agent', 'Unknown')}"
        )

        return response


def add_logging_middleware(app):
    """
    Add logging middleware to the FastAPI app
    """
    app.add_middleware(LoggingMiddleware)
    return app