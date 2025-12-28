from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
from pydantic_settings import BaseSettings
import os

# Load environment variables
load_dotenv()

class Settings(BaseSettings):
    # Agent configuration
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")

    # Qdrant configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")

    # Database configuration
    database_url: str = os.getenv("DATABASE_URL", "")
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")

    # Agent timeout
    agent_timeout: int = int(os.getenv("AGENT_TIMEOUT", "30"))

    # Logging level
    log_level: str = os.getenv("LOG_LEVEL", "INFO")

    class Config:
        env_file = ".env"
        extra = "ignore"

# Initialize settings
settings = Settings()

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize database connection
    import logging
    from utils.database import init_database, close_database
    logging.info("Starting up application...")
    await init_database()
    logging.info("Application startup completed, database connection initialized")
    yield
    # Shutdown: Close database connection
    logging.info("Shutting down application...")
    await close_database()
    logging.info("Application shutdown completed, database connection closed")

# Initialize FastAPI app with lifespan
app = FastAPI(
    title="Backend Agent Integration API",
    description="API for connecting the ChatKit UI to the existing RAG agent, with interaction logging to Neon database",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS middleware for ChatKit UI
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:4000",
        "http://127.0.0.1:4000",
        "http://localhost:8000",
        "http://127.0.0.1:8000",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for client-side access
    expose_headers=["Access-Control-Allow-Origin"]
)

# Include routers
import sys
import os
# Add the current directory to the path to allow relative imports
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from routers import chat, profile
# from src.api.auth import router as auth_router # Deprecated
from src.api.conversations import router as conversations_router
from utils.logging import setup_logging_middleware
from utils.error_handlers import setup_error_handlers

# Include the chat router with API versioning
app.include_router(chat.router, prefix="/api/v1", tags=["chat"])

# Include the profile router (Task says /api/profile/onboarding)
app.include_router(profile.router, prefix="/api/profile", tags=["profile"])

# Include the conversations router with API versioning
app.include_router(conversations_router, prefix="/api/v1", tags=["conversations"])

# Setup logging and error handling
app = setup_logging_middleware(app)
app = setup_error_handlers(app)

@app.get("/")
async def root():
    return {"message": "Backend Agent Integration API"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}