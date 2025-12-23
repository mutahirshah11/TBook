# Data Model: Database Connection and User Data Storage

## Entity: User
**Description**: Represents a registered user account, with data structure prepared for future Better Auth integration

**Fields**:
- `id` (string/UUID): Unique identifier for the user
- `email` (string): User's email address (unique, validated format)
- `password_hash` (string): Securely hashed password using industry-standard algorithm
- `created_at` (timestamp): Account creation timestamp

**Validation rules**:
- Email must be in valid format and unique
- Password must be securely hashed (not stored in plain text)
- Created timestamp automatically set on creation

**Relationships**:
- One-to-many with Conversation entity (one user can have many conversations)

**Preparation for Better Auth**:
- Schema designed to support future Better Auth integration
- Fields aligned with Better Auth expected data structure

## Entity: Conversation
**Description**: Represents a single conversation entry in user history

**Fields**:
- `id` (string/UUID): Unique identifier for the conversation
- `user_id` (string/UUID): Foreign key referencing the User
- `query` (text): The user's query/input to the chatbot
- `response` (text): The chatbot's response to the query
- `timestamp` (timestamp): When the conversation occurred
- `created_at` (timestamp): Record creation time

**Validation rules**:
- User_id must reference an existing user
- Query and response fields must not be empty
- Timestamp automatically set to current time
- Automatically deleted when exceeding 50 per user limit

**Relationships**:
- Many-to-one with User entity (many conversations belong to one user)

## Constraints
- Each user can have maximum 50 conversations stored
- Conversations are automatically deleted in FIFO order when limit is exceeded
- User email must be unique across the system
- Passwords are never stored in plain text