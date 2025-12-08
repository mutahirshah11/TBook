from qdrant_utils import get_qdrant_client, create_collection_if_not_exists
from qdrant_client import models

COLLECTION_NAME = "book_vectors"
VECTOR_SIZE = 1024
DISTANCE_METRIC = models.Distance.COSINE

def setup_qdrant_collection():
    """
    Connects to Qdrant and ensures the collection exists with the correct configuration.
    """
    try:
        client = get_qdrant_client()
        create_collection_if_not_exists(client, COLLECTION_NAME, VECTOR_SIZE, DISTANCE_METRIC)
    except Exception as e:
        print(f"Error setting up Qdrant collection: {e}")

if __name__ == "__main__":
    setup_qdrant_collection()
