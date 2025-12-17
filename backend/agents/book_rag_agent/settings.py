import os
import logging
from typing import Optional

from dotenv import load_dotenv
from qdrant_client import QdrantClient
import cohere
from openai import AsyncOpenAI

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class Settings:
    """Application settings and client initialization."""
    
    # Environment Variables
    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    COHERE_API_KEY: Optional[str] = os.getenv("COHERE_API_KEY")
    GEMINI_API_KEY: Optional[str] = os.getenv("GEMINI_API_KEY")
    
    # Constants
    COLLECTION_NAME: str = "book_vectors"
    EMBEDDING_MODEL: str = "embed-english-v3.0"
    
    # Clients (Lazily initialized)
    _qdrant_client: Optional[QdrantClient] = None
    _cohere_client: Optional[cohere.Client] = None
    _openai_client: Optional[AsyncOpenAI] = None

    @classmethod
    def get_qdrant_client(cls) -> QdrantClient:
        if cls._qdrant_client is None:
            if not cls.QDRANT_URL:
                logger.warning("QDRANT_URL not set. Qdrant client may fail.")
            cls._qdrant_client = QdrantClient(
                url=cls.QDRANT_URL,
                api_key=cls.QDRANT_API_KEY
            )
        return cls._qdrant_client

    @classmethod
    def get_cohere_client(cls) -> cohere.Client:
        if cls._cohere_client is None:
            if not cls.COHERE_API_KEY:
                logger.warning("COHERE_API_KEY not set. Embeddings will fail.")
                # We return None or let it fail downstream. 
                # Ideally, we should raise an error if strictly required.
            cls._cohere_client = cohere.Client(cls.COHERE_API_KEY)
        return cls._cohere_client
        
    @classmethod
    def get_openai_client(cls) -> AsyncOpenAI:
        """
        Returns an AsyncOpenAI client configured for Gemini (via OpenAI compatibility).
        """
        if cls._openai_client is None:
            if not cls.GEMINI_API_KEY:
                logger.warning("GEMINI_API_KEY not set.")
                
            # Gemini OpenAI Compatibility Endpoint
            base_url = "https://generativelanguage.googleapis.com/v1beta/openai/"
            
            cls._openai_client = AsyncOpenAI(
                api_key=cls.GEMINI_API_KEY,
                base_url=base_url
            )
        return cls._openai_client

settings = Settings()
