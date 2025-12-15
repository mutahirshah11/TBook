import argparse
import json
import os
import time
from typing import List, Dict, Any

import cohere
from dotenv import load_dotenv
from requests.exceptions import RequestException

from models import Chunk, VectorRecord # Assuming models.py is in the same directory

# Load environment variables
load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
if not COHERE_API_KEY:
    raise ValueError("COHERE_API_KEY environment variable not set.")

COHERE_MODEL_NAME = "embed-english-v3.0"
VECTOR_DIMENSION = 1024 # As clarified in spec

def get_cohere_client() -> cohere.Client:
    """
    Initializes and returns a Cohere client instance.
    """
    return cohere.Client(COHERE_API_KEY)

def generate_embeddings_with_retry(
    texts: List[str], client: cohere.Client, max_retries: int = 5, initial_delay: int = 1
) -> List[List[float]]:
    """
    Generates embeddings for a list of texts with retry logic for rate limits.
    """
    for attempt in range(max_retries):
        try:
            response = client.embed(
                texts=texts,
                model=COHERE_MODEL_NAME,
                input_type="search_document" # Or "search_query" depending on use case
            )
            embeddings = response.embeddings
            # Ensure correct dimension
            for emb in embeddings:
                if len(emb) != VECTOR_DIMENSION:
                    raise ValueError(f"Cohere returned embedding of size {len(emb)}, expected {VECTOR_DIMENSION}")
            return embeddings
        except cohere.CohereAPIError as e:
            if "too many requests" in str(e).lower() or "rate limit" in str(e).lower():
                delay = initial_delay * (2 ** attempt)
                print(f"Rate limit hit. Retrying in {delay} seconds... (Attempt {attempt + 1}/{max_retries})")
                time.sleep(delay)
            else:
                raise e
        except RequestException as e: # Catch network-related errors
            delay = initial_delay * (2 ** attempt)
            print(f"Network error: {e}. Retrying in {delay} seconds... (Attempt {attempt + 1}/{max_retries})")
            time.sleep(delay)
    raise Exception(f"Failed to generate embeddings after {max_retries} attempts.")


def read_chunks_from_json(input_path: str) -> List[Chunk]:
    """
    Reads chunk data from a JSON file and converts it into a list of Chunk objects.
    """
    with open(input_path, 'r', encoding='utf-8') as f:
        chunks_data = json.load(f)
    
    chunks = [Chunk(**data) for data in chunks_data]
    print(f"Read {len(chunks)} chunks from {input_path}")
    return chunks

def main():
    parser = argparse.ArgumentParser(description="Generates embeddings for text chunks using Cohere API.")
    parser.add_argument("--input", type=str, default="chunks.json",
                        help="Path to the input JSON file containing chunks.")
    parser.add_argument("--output", type=str, default="embeddings.json",
                        help="Path to save the generated embeddings JSON.")
    args = parser.parse_args()

    chunks = read_chunks_from_json(args.input)
    cohere_client = get_cohere_client()
    
    embedded_records: List[Dict[str, Any]] = []
    batch_size = 96 # Cohere API batch limit is typically 96 texts per request

    for i in range(0, len(chunks), batch_size):
        batch_chunks = chunks[i : i + batch_size]
        batch_texts = [chunk.content for chunk in batch_chunks]

        print(f"Generating embeddings for batch {i//batch_size + 1}/{(len(chunks)-1)//batch_size + 1}...")
        embeddings = generate_embeddings_with_retry(batch_texts, cohere_client)

        for chunk, embedding in zip(batch_chunks, embeddings):
            payload = {
                "chunk_id": chunk.id,
                "content": chunk.content,
                "file_path": chunk.file_path,
                "heading": chunk.heading,
                "chapter": chunk.chapter
            }
            # Create a dictionary matching the VectorRecord structure for JSON serialization
            embedded_records.append({
                "id": chunk.id,
                "vector": embedding,
                "payload": payload
            })
    
    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump(embedded_records, f, indent=2)
    print(f"Generated {len(embedded_records)} embeddings and saved to {args.output}")

if __name__ == "__main__":
    main()
