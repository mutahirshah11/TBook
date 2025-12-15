import hashlib
import uuid
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from models import Chunk

def calculate_content_hash(chunk: "Chunk") -> str:
    """
    Calculates a deterministic UUID based on the chunk content.
    This serves as a unique ID for idempotency in Qdrant, which requires UUIDs or integers.
    """
    # Combine relevant fields to create a unique string for hashing
    hash_string = f"{chunk.file_path}-{chunk.heading}-{chunk.chapter}-{chunk.content}"
    
    # Use MD5 to get a 128-bit hash, which fits perfectly into a UUID
    md5_hash = hashlib.md5(hash_string.encode('utf-8')).hexdigest()
    
    # Convert hex to a standard UUID string format (e.g., '550e8400-e29b-41d4-a716-446655440000')
    return str(uuid.UUID(md5_hash))

# Example usage (will be used by other scripts)
# from models import Chunk
# chunk_data = Chunk(...)
# chunk_data.id = calculate_content_hash(chunk_data)
