# Data Model: Better-Auth Authentication System

## Entities

### User (Better-Auth managed)
**Source**: Better-Auth default user model
- `id` (string): Unique user identifier (UUID format)
- `email` (string): User's email address (unique, required)
- `emailVerified` (timestamp): When email was verified (nullable)
- `firstName` (string): User's first name (nullable)
- `lastName` (string): User's last name (nullable)
- `createdAt` (timestamp): Account creation time
- `updatedAt` (timestamp): Last update time
- `image` (string): Profile image URL (nullable)
- `disabled` (boolean): Account status (default: false)

### UserProfile (Custom extension)
**Purpose**: Extended user profile data for personalization
- `id` (string): Primary key (UUID format)
- `userId` (string): Foreign key to Better-Auth User.id (unique, required)
- `pythonProficiency` (string): Enum values ["Beginner", "Intermediate", "Advanced"] (required)
- `developerRole` (string): Enum values ["Frontend Developer", "Backend Developer", "Full Stack Developer", "None"] (required)
- `createdAt` (timestamp): Profile creation time
- `updatedAt` (timestamp): Last profile update time

### Session (Better-Auth managed)
**Source**: Better-Auth default session model
- `id` (string): Unique session identifier
- `userId` (string): Foreign key to User.id
- `expiresAt` (timestamp): Session expiration time
- `createdAt` (timestamp): Session creation time
- `updatedAt` (timestamp): Last update time
- `ipAddress` (string): IP address of session creator (nullable)
- `userAgent` (string): User agent string (nullable)

## Relationships

### UserProfile → User
- **Type**: One-to-One (required)
- **Constraint**: `UserProfile.userId` references `User.id`
- **Cascade**: Delete UserProfile when User is deleted
- **Index**: `UserProfile.userId` is indexed for performance

## Validation Rules

### User
- Email must be valid email format
- Email must be unique across all users
- First name and last name must be 1-50 characters if provided

### UserProfile
- `pythonProficiency` must be one of the defined enum values
- `developerRole` must be one of the defined enum values
- `userId` must reference an existing user
- Each user can have only one profile (enforced by unique constraint on userId)

### Session
- Session must have valid expiration time
- Session must reference an existing user
- Expired sessions should be automatically cleaned up

## State Transitions

### User Account States
- `active`: User can authenticate and access system
- `disabled`: User cannot authenticate (set by admin or user request)
- `pending_verification`: Email not yet verified (if email verification required)

### Session States
- `active`: Session is valid and can be used
- `expired`: Session has passed expiration time
- `revoked`: Session was manually invalidated

## Access Patterns

### Authentication
- Query user by email for login verification
- Validate session by ID and check expiration
- Lookup user profile by user ID for personalization

### Personalization
- Retrieve user profile to customize chatbot responses
- Access profile data without loading full user object when possible

### Security
- Audit trail for authentication attempts
- Session management for concurrent access control
- Profile update tracking for compliance requirements