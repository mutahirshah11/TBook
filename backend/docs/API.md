# API Documentation

## Base URL
```
https://your-deployment-url.com/api/v1
```

## Authentication
This API does not require authentication for basic functionality. All requests are stateless and do not require session management.

## Common Headers
- `Content-Type: application/json` - Required for POST requests
- `Accept: application/json` - For standard responses
- `Accept: text/event-stream` - For streaming responses

## Endpoints

### Chat API
#### POST /chat
Send a message to the agent and receive a complete response.

**Request Body**:
```json
{
  "message": {
    "content": "string (required, 1-4000 characters)",
    "user_id": "string (optional, alphanumeric with hyphens/underscores)",
    "timestamp": "ISO 8601 datetime (optional, defaults to now)",
    "metadata": "object (optional, additional context for the agent)"
  },
  "stream": "boolean (optional, defaults to false)"
}
```

**Success Response (200)**:
```json
{
  "response": {
    "content": "string",
    "message_id": "string",
    "timestamp": "ISO 8601 datetime",
    "status": "string (success|error|partial)"
  },
  "status_code": "integer",
  "error": "object (optional)"
}
```

**Validation Error Response (422)**:
```json
{
  "error": "validation_error",
  "message": "Request validation failed",
  "details": {
    "errors": [
      {
        "field": "string",
        "message": "string",
        "type": "string"
      }
    ]
  }
}
```

**Server Error Response (500)**:
```json
{
  "error": "internal_server_error",
  "message": "An internal server error occurred",
  "details": {
    "error_type": "string"
  }
}
```

### Streaming Chat API
#### POST /chat/stream
Send a message to the agent and receive a streaming response via Server-Sent Events.

**Request Body**:
Same as POST /chat

**Success Response (200)**:
Content-Type: `text/event-stream`
```
data: {"content": "partial response", "status": "partial"}

data: {"content": "final response", "status": "success"}
```

**Error in Stream**:
```
data: {"content": "error message", "status": "error"}
```

## Error Codes

| Code | Description |
|------|-------------|
| 200 | Success |
| 400 | Bad Request - Invalid request format |
| 422 | Validation Error - Request validation failed |
| 500 | Internal Server Error - Something went wrong on the server |

## Rate Limiting
No explicit rate limiting is implemented. In production, rate limiting should be handled at the infrastructure level.

## Request/Response Limits
- Maximum message content: 4000 characters
- Maximum request size: Limited by serverless platform constraints
- Timeout: Configurable via AGENT_TIMEOUT environment variable (default 30 seconds)