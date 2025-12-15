from qdrant_client import QdrantClient, models
import os
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

def get_qdrant_client() -> QdrantClient:
    """
    Initializes and returns a QdrantClient instance.
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables.")

    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    return client

def create_collection_if_not_exists(client: QdrantClient, collection_name: str, vector_size: int, distance: models.Distance):
    """
    Creates a Qdrant collection if it does not already exist.
    """
    if not client.collection_exists(collection_name=collection_name):
        print(f"Collection '{collection_name}' does not exist. Creating now...")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=distance),
            # on_disk_payload=True # As per data-model.md, can be optional for performance
        )
        print(f"Collection '{collection_name}' created successfully.")
    else:
        print(f"Collection '{collection_name}' already exists.")

if __name__ == "__main__":
    # Example usage:
    try:
        qdrant_client = get_qdrant_client()
        create_collection_if_not_exists(
            client=qdrant_client,
            collection_name="book_vectors", # As per spec
            vector_size=1024,              # As per spec
            distance=models.Distance.COSINE # As per spec
        )
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
