# Data Model: Qdrant Book Vectors

## Entities

### 1. SourceDocument
Represents a raw markdown file from the Docusaurus documentation.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| `file_path` | `string` | Absolute or relative path to the file. | Must exist on disk. Ends in `.md`. |
| `content` | `string` | Raw text content of the file. | Not empty. |
| `chapter` | `string` | derived from directory structure or filename. | - |

### 2. Chunk
A processed segment of text, ready for embedding.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| `chunk_id` | `string` | Unique identifier (hash of content + metadata). | UUID or Hash string. |
| `content` | `string` | The actual text segment. | 500-1000 tokens approx. |
| `source_file` | `string` | Path to source document. | - |
| `heading` | `string` | The nearest preceding header (H1-H3). | - |
| `chapter` | `string` | Chapter name. | - |
| `token_count` | `integer` | Number of tokens in this chunk. | > 0 |

### 3. VectorRecord
The data structure stored in Qdrant.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| `id` | `string` | The deterministic hash ID. | Valid Qdrant ID format (UUID/int). |
| `vector` | `list[float]` | The embedding vector. | Length = 1024. |
| `payload` | `object` | Metadata for retrieval. | JSON object. |

#### Payload Schema
```json
{
  "chunk_id": "string",
  "content": "string",
  "file_path": "string",
  "heading": "string",
  "chapter": "string"
}
```

## Storage

### Qdrant Collection: `book_vectors`
- **Metric**: Cosine
- **Dimension**: 1024
- **on_disk_payload**: true (optional, for performance)
