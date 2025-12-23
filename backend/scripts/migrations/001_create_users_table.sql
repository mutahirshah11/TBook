-- Migration: Create users table for authentication data storage
-- This table is prepared for future Better Auth integration

CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create index for efficient email lookups
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Add comments for documentation
COMMENT ON TABLE users IS 'User authentication data, prepared for Better Auth integration';
COMMENT ON COLUMN users.email IS 'User email address, used for identification';
COMMENT ON COLUMN users.password_hash IS 'Securely hashed password using industry-standard algorithm';
COMMENT ON COLUMN users.created_at IS 'Account creation timestamp';