import sys
import os
# Add the backend directory to the path to allow imports
backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, backend_dir)

from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from pydantic import ValidationError
from typing import Union
import logging
import traceback
from schemas import ErrorResponse


async def http_exception_handler(request: Request, exc: HTTPException):
    """
    Handle HTTP exceptions
    """
    logging.error(f"HTTP Exception: {exc.status_code} - {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": "http_exception",
            "message": str(exc.detail),
            "details": {"status_code": exc.status_code}
        }
    )


async def validation_exception_handler(request: Request, exc: ValidationError):
    """
    Handle validation exceptions from Pydantic
    """
    logging.error(f"Validation Error: {exc}")
    errors = []
    for error in exc.errors():
        errors.append({
            "field": error.get("loc", ["unknown"])[-1],
            "message": error.get("msg", "Validation error"),
            "type": error.get("type", "validation_error")
        })

    return JSONResponse(
        status_code=422,
        content={
            "error": "validation_error",
            "message": "Request validation failed",
            "details": {"errors": errors}
        }
    )


async def general_exception_handler(request: Request, exc: Union[Exception, Exception]):
    """
    Handle general exceptions
    """
    logging.error(f"General Exception: {str(exc)}")
    logging.error(traceback.format_exc())

    return JSONResponse(
        status_code=500,
        content={
            "error": "internal_server_error",
            "message": "An internal server error occurred",
            "details": {"error_type": type(exc).__name__}
        }
    )


def setup_error_handlers(app):
    """
    Setup error handlers for the FastAPI app
    """
    app.add_exception_handler(HTTPException, http_exception_handler)
    app.add_exception_handler(ValidationError, validation_exception_handler)
    app.add_exception_handler(Exception, general_exception_handler)

    return app