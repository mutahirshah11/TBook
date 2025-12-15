# Data Model: Backend Agent Integration

## Key Entities

### Message
**Description**: Represents a chat message sent from the UI to the agent
**Fields**:
- `content` (string): The actual message content from the user
- `user_id` (string, optional): Identifier for the user sending the message
- `timestamp` (datetime): When the message was sent
- `metadata` (object, optional): Additional context or parameters for the agent

**Validation Rules**:
- Content must not be empty
- Content length must be between 1 and 4000 characters
- User_id must follow UUID format if provided

### Response
**Description**: Represents the agent's response to a user message
**Fields**:
- `content` (string): The agent's response content
- `message_id` (string): Reference to the original message
- `timestamp` (datetime): When the response was generated
- `status` (string): Status of the response (success, error, partial)

**Validation Rules**:
- Content must not be empty for successful responses
- Status must be one of: "success", "error", "partial"
- Message_id must reference a valid message

### ChatRequest
**Description**: API request payload for the chat endpoint
**Fields**:
- `message` (Message): The message object to send to the agent
- `stream` (boolean, optional): Whether to use streaming response (default: false)

**Validation Rules**:
- Message object must be valid
- Stream parameter must be boolean if provided

### ChatResponse
**Description**: API response for the chat endpoint
**Fields**:
- `response` (Response): The agent's response
- `status_code` (integer): HTTP status code
- `error` (object, optional): Error details if request failed

**Validation Rules**:
- Response must be present for successful requests
- Error must be present for failed requests
- Status code must be valid HTTP status code

### InteractionLog
**Description**: Represents logged interaction metadata for analytics in Neon database
**Fields**:
- `id` (string): Unique identifier for the log entry
- `user_id` (string, optional): User identifier if available
- `session_id` (string, optional): Session identifier for grouping related interactions
- `request_content` (string): The original user message content
- `response_content` (string): The agent's response content
- `request_timestamp` (datetime): When the request was received
- `response_timestamp` (datetime): When the response was completed
- `response_time_ms` (integer): Time taken for the agent to respond
- `agent_model` (string, optional): The model used by the agent
- `error_occurred` (boolean): Whether an error occurred during processing
- `error_message` (string, optional): Error message if an error occurred
- `metadata` (object, optional): Additional metadata for analytics

**Validation Rules**:
- All timestamps must be valid
- Response_time_ms must be non-negative
- Error_occurred must be boolean

## State Transitions

### Message Lifecycle
1. **Created**: Message is received by the backend from the UI
2. **Forwarded**: Message is sent to the agent for processing
3. **Processed**: Agent has completed processing the message
4. **Responded**: Response has been returned to the UI

### Response Lifecycle
1. **Pending**: Agent is processing the message
2. **Streaming** (for streaming requests): Response chunks are being sent
3. **Complete**: Full response has been delivered
4. **Error**: An error occurred during processing

## Relationships
- ChatRequest contains one Message
- ChatResponse contains one Response
- Response references the original Message via message_id
- InteractionLog records each complete request-response cycle