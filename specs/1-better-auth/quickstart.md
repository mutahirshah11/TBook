# Quickstart Guide: Better-Auth Authentication System

## Prerequisites

- Python 3.11+
- Node.js 18+
- PostgreSQL (or Neon Serverless Postgres)
- Git

## Setup

### 1. Environment Configuration

```bash
# Copy environment template
cp .env.example .env

# Update the following variables in .env:
BETTER_AUTH_SECRET=your-secret-key-here
NEON_DATABASE_URL=your-neon-database-url
NEXTAUTH_URL=http://localhost:3000
```

### 2. Backend Setup

```bash
# Navigate to backend directory
cd backend

# Install Python dependencies
pip install -r requirements.txt

# Run database migrations
python -m src.auth.better_auth_integration --migrate

# Start the backend server
uvicorn main:app --reload --port 8000
```

### 3. Frontend Setup

```bash
# Navigate to frontend directory
cd frontend

# Install Node.js dependencies
npm install

# Start the frontend development server
npm run dev
```

## Key Features

### Authentication Flows

1. **User Registration**:
   - Navigate to `/auth/signup`
   - Provide email, password, and profile information
   - Account is created with extended profile data

2. **User Login**:
   - Navigate to `/auth/signin`
   - Enter credentials to authenticate
   - Session is established for personalized experience

3. **Profile Management**:
   - Access `/profile` to view/edit profile
   - Update Python proficiency and developer role
   - Changes affect personalization immediately

### API Endpoints

- `POST /api/auth/signup` - Create new user with profile
- `POST /api/auth/signin` - Authenticate user
- `POST /api/auth/signout` - End user session
- `GET /api/auth/me` - Get current user and profile

## Testing

### Run Backend Tests

```bash
# Unit tests
python -m pytest tests/unit/

# Integration tests
python -m pytest tests/integration/

# Contract tests
python -m pytest tests/contract/
```

### Run Frontend Tests

```bash
# Unit tests
npm run test:unit

# Integration tests
npm run test:integration
```

## Development

### Adding New Authentication Methods

1. Update `src/auth/better_auth_integration.py` to configure additional providers
2. Update the API contracts in `specs/1-better-auth/contracts/`
3. Add corresponding frontend components in `frontend/src/components/auth/`
4. Write tests for the new authentication method

### Extending User Profile

1. Update the `UserProfile` model in `src/models/user_profile.py`
2. Update the database migration scripts
3. Update API contracts to reflect new fields
4. Add frontend form elements for new profile fields
5. Update personalization logic to use new profile data

## Troubleshooting

### Common Issues

1. **Database Connection Errors**:
   - Verify `NEON_DATABASE_URL` is correctly set
   - Check that your database is accessible
   - Run migrations if schema changes are needed

2. **Authentication Not Working**:
   - Ensure `BETTER_AUTH_SECRET` is the same across services
   - Check that session cookies are being set properly
   - Verify CORS settings are correct

3. **Profile Data Not Saving**:
   - Confirm that the user is properly authenticated
   - Check that profile creation happens after user creation
   - Verify database constraints are not being violated