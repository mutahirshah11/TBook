-- Migration: Create conversations table for conversation history storage
-- Each user can have maximum 50 conversations stored

CREATE TABLE IF NOT EXISTS conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Index for efficient user conversation lookup
CREATE INDEX IF NOT EXISTS idx_conversations_user_id ON conversations(user_id);

-- Index for conversation ordering by time
CREATE INDEX IF NOT EXISTS idx_conversations_timestamp ON conversations(timestamp);

-- Add comments for documentation
COMMENT ON TABLE conversations IS 'User conversation history, limited to last 50 per user';
COMMENT ON COLUMN conversations.user_id IS 'Foreign key referencing the user';
COMMENT ON COLUMN conversations.query IS 'The user''s query/input to the chatbot';
COMMENT ON COLUMN conversations.response IS 'The chatbot''s response to the query';
COMMENT ON COLUMN conversations.timestamp IS 'When the conversation occurred';