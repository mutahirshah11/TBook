# Data Model: Docusaurus RAG Chatbot UI

## Core Entities

### ConversationSession
- **id**: string (unique identifier for the session)
- **messages**: Message[] (ordered array of messages in the conversation)
- **createdAt**: Date (timestamp when session was created)
- **lastActiveAt**: Date (timestamp of last message)
- **isActive**: boolean (whether the session is currently open)

### Message
- **id**: string (unique identifier for the message)
- **content**: string (the text content of the message)
- **sender**: 'user' | 'ai' (who sent the message)
- **timestamp**: Date (when the message was sent)
- **context**: string | null (optional context from selected text)
- **status**: 'sending' | 'delivered' | 'error' (delivery status for user messages)
- **sourceReferences**: string[] (references to book sections for AI responses)

### ChatConfig
- **theme**: 'light' | 'dark' | 'auto' (current theme setting)
- **isExpanded**: boolean (whether chat window is open)
- **position**: { x: number, y: number } (coordinates for floating button)
- **sessionTimeout**: number (minutes of inactivity before session clears)

## State Transitions

### ConversationSession
- **Created** → **Active** (when user opens chat for first time in session)
- **Active** → **Inactive** (when user closes chat or session times out)
- **Inactive** → **Archived** (when browser session ends)

### Message
- **Created** → **Sending** (when user submits message)
- **Sending** → **Delivered** (when message is successfully sent to backend)
- **Sending** → **Error** (when there's a network or backend error)

## Validation Rules

### Message
- Content must be 1-2000 characters
- Must have valid sender type ('user' or 'ai')
- Timestamp must be within current session window

### ConversationSession
- Cannot have more than 100 messages (for performance)
- Must be associated with a valid browser session
- Cannot be active for more than 24 hours

## Relationships
- One ConversationSession contains many Messages (1 to many)
- One Message optionally references many Book Sections (via sourceReferences)