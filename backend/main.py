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

# Initialize settings
settings = Settings()

# Initialize FastAPI app
app = FastAPI(
    title="Backend Agent Integration API",
    description="API for connecting the ChatKit UI to the existing RAG agent, with interaction logging to Neon database",
    version="1.0.0"
)

# Configure CORS middleware for ChatKit UI
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
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

from routers import chat
from utils.logging import setup_logging_middleware
from utils.error_handlers import setup_error_handlers

# Include the chat router with API versioning
app.include_router(chat.router, prefix="/api/v1", tags=["chat"])

# Setup logging and error handling
app = setup_logging_middleware(app)
app = setup_error_handlers(app)

@app.get("/")
async def root():
    return {"message": "Backend Agent Integration API"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}