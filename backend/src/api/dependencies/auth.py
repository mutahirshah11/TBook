from fastapi import Request, HTTPException, Depends
from utils.database import neon_db
import logging
from typing import Optional, Dict, Any
from urllib.parse import unquote

async def get_current_user(request: Request) -> Dict[str, Any]:
    """
    Validate the session token from the 'better-auth.session_token' cookie.
    Returns the user dictionary if valid, including profile data.
    Raises 401 if invalid or missing.
    """
    token = request.cookies.get("better-auth.session_token")
    if token:
        # Better Auth tokens are usually alphanumeric but can be URL encoded in cookies
        # They are also signed with a signature appended after a dot (token.signature)
        token = unquote(token).strip()
        if "." in token:
            token = token.split(".")[0]
    
    if not token:
        # Check Authorization header (Bearer token) as fallback
        auth_header = request.headers.get("Authorization")
        if auth_header and auth_header.startswith("Bearer "):
            token = unquote(auth_header.split(" ")[1]).strip()
            # Handle signed tokens in header too, just in case
            if "." in token:
                token = token.split(".")[0]
            
    if not token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    pool = neon_db.pool
    if not pool:
        logging.error("Database connection not available")
        raise HTTPException(status_code=503, detail="Database unavailable")

    # Query session and user data
    # We use quoted identifiers for Better Auth tables and columns
    # We explicitly use public. schema to avoid any ambiguity
    query = """
        SELECT u.id, u.email, u.name, u.image, s."expiresAt", 
               p.python_proficiency, p.developer_role, p.is_onboarded
        FROM public."session" s
        JOIN public."user" u ON s."userId" = u.id
        LEFT JOIN public.user_profile p ON u.id = p.user_id
        WHERE s.token = $1 AND s."expiresAt" > NOW()
    """
    
    try:
        row = await pool.fetchrow(query, token)
        if not row:
            raise HTTPException(status_code=401, detail="Invalid or expired session")
            
        return dict(row)
    except Exception as e:
        if isinstance(e, HTTPException):
            raise e
        logging.error(f"Auth validation error: {e}")
        raise HTTPException(status_code=401, detail="Authentication failed")

async def require_onboarded_user(user: Dict[str, Any] = Depends(get_current_user)) -> Dict[str, Any]:
    """
    Dependency that requires the user to be fully onboarded.
    Raises 403 if is_onboarded is False.
    """
    if not user.get("is_onboarded"):
        raise HTTPException(status_code=403, detail="Profile Incomplete")
    return user