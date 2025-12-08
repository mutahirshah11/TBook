import argparse
import json
import os
from typing import List, Dict, Any

from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

from qdrant_utils import get_qdrant_client, create_collection_if_not_exists # Assuming qdrant_utils.py is in the same directory

# Load environment variables
load_dotenv()

COLLECTION_NAME = "book_vectors"
VECTOR_SIZE = 1024
DISTANCE_METRIC = models.Distance.COSINE

def read_embeddings_from_json(input_path: str) -> List[Dict[str, Any]]:
    """
    Reads embedding data from a JSON file.
    """
    with open(input_path, 'r', encoding='utf-8') as f:
        embeddings_data = json.load(f)
    print(f"Read {len(embeddings_data)} embeddings from {input_path}")
    return embeddings_data

def main():
    parser = argparse.ArgumentParser(description="Uploads generated embeddings to Qdrant Cloud.")
    parser.add_argument("--input", type=str, default="embeddings.json",
                        help="Path to the input JSON file containing embeddings.")
    args = parser.parse_args()

    # Ensure collection exists
    try:
        client = get_qdrant_client()
        create_collection_if_not_exists(client, COLLECTION_NAME, VECTOR_SIZE, DISTANCE_METRIC)
    except Exception as e:
        print(f"Failed to setup Qdrant collection: {e}")
        return

    embeddings_data = read_embeddings_from_json(args.input)

    # Prepare points for upsert
    points = []
    for record in embeddings_data:
        points.append(
            models.PointStruct(
                id=record["id"],
                vector=record["vector"],
                payload=record["payload"]
            )
        )
    
    # Perform batch upsert
    batch_size = 100 # Can be adjusted based on Qdrant recommendations and network conditions
    for i in range(0, len(points), batch_size):
        batch_points = points[i : i + batch_size]
        print(f"Upserting batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1} ({len(batch_points)} points)...")
        operation_info = client.upsert(
            collection_name=COLLECTION_NAME,
            wait=True, # Wait for the operation to be completed
            points=batch_points
        )
        print(f"Batch upsert operation info: {operation_info}")
    
    print(f"Successfully uploaded {len(points)} vectors to collection '{COLLECTION_NAME}'.")

if __name__ == "__main__":
    main()
