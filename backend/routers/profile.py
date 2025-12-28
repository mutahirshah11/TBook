from fastapi import APIRouter, Depends, HTTPException
from backend.src.api.dependencies.auth import get_current_user, require_onboarded_user
from backend.src.models.profile import UserProfile, PythonProficiency, DeveloperRole
from utils.database import neon_db
from pydantic import BaseModel
import logging

router = APIRouter()

class OnboardingRequest(BaseModel):
    python_proficiency: PythonProficiency
    developer_role: DeveloperRole

@router.get("/me")
async def get_my_profile(current_user: dict = Depends(get_current_user)):
    """
    Get current user profile (lax auth).
    Used by frontend to check onboarding status.
    """
    return current_user

@router.post("/onboarding")
async def complete_onboarding(
    data: OnboardingRequest,
    current_user: dict = Depends(get_current_user)
):
    user_id = current_user["id"]
    pool = neon_db.pool
    
    if not pool:
        raise HTTPException(status_code=503, detail="Database unavailable")

    if current_user.get("is_onboarded"):
        raise HTTPException(status_code=403, detail="Profile already completed")

    try:
        await pool.execute("""
            INSERT INTO user_profile (user_id, python_proficiency, developer_role, is_onboarded, updated_at)
            VALUES ($1, $2, $3, TRUE, NOW())
            ON CONFLICT (user_id) 
            DO UPDATE SET 
                python_proficiency = EXCLUDED.python_proficiency,
                developer_role = EXCLUDED.developer_role,
                is_onboarded = TRUE,
                updated_at = NOW()
        """, user_id, data.python_proficiency.value, data.developer_role.value)
        
        logging.info(f"User {user_id} completed onboarding")
        return {"status": "success", "user_id": user_id}
        
    except Exception as e:
        logging.error(f"Onboarding failed for {user_id}: {e}")
        # Return the actual error string for debugging
        raise HTTPException(status_code=500, detail=f"Failed to save profile: {str(e)}")