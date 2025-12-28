from pydantic import BaseModel
from typing import Optional
from enum import Enum
from datetime import datetime

class PythonProficiency(str, Enum):
    BEGINNER = "Beginner"
    INTERMEDIATE = "Intermediate"
    ADVANCED = "Advanced"

class DeveloperRole(str, Enum):
    FRONTEND = "Frontend Developer"
    BACKEND = "Backend Developer"
    FULL_STACK = "Full Stack Developer"
    NONE = "None"

class UserProfile(BaseModel):
    user_id: str
    python_proficiency: PythonProficiency
    developer_role: DeveloperRole
    is_onboarded: bool = False
    created_at: datetime
    updated_at: datetime
