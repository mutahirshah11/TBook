# Quickstart Guide: Implementing User Auth & Profile

## Prerequisites
- Node.js v20+ (for Auth Server)
- Python 3.11+ (for Backend)
- Neon Postgres Connection String

## Step 1: Database Migration
1. Create the `user_profile` table in Neon.
2. (Better-Auth will auto-migrate its own tables on startup, but ensure connection is valid).

```sql
CREATE TABLE IF NOT EXISTS user_profile (
    user_id TEXT PRIMARY KEY REFERENCES "user"(id),
    python_proficiency TEXT NOT NULL,
    developer_role TEXT NOT NULL,
    is_onboarded BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);
```

## Step 2: Setup Node.js Auth Server
1. Initialize `auth-server/` directory.
2. Install dependencies: `better-auth`, `pg`, `hono` (or express).
3. Configure `better-auth` with Neon connection.
4. Expose API at `http://localhost:4000/api/auth/*`.

## Step 3: Integrate Docusaurus
1. Install `@better-auth/client`.
2. Create `src/components/AuthProvider.tsx`.
3. Wrap application root.
4. Create `src/components/ProtectedRoute.tsx`.
5. Implement `src/pages/onboarding.tsx`.

## Step 4: Python Backend Middleware
1. Add middleware to check `session_token` cookie.
2. Query DB to validate session and check `user_profile`.
3. Reject requests if profile missing.

## Testing
1. **Signup**: Create new user via UI. Check DB `user` table.
2. **Onboarding**: Redirects to `/onboarding`.
3. **Submit**: Submit form. Check DB `user_profile`.
4. **Access**: Access `/dashboard`. Should succeed.
