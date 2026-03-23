"""
Pydantic schemas for authentication validation
"""
from pydantic import BaseModel, EmailStr, field_validator, ConfigDict
from typing import Optional
from enum import Enum


class SoftwareLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class HardwareAccess(str, Enum):
    NONE = "none"
    BASIC = "basic"
    ADVANCED = "advanced"


class TechnicalComfort(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class SignupRequest(BaseModel):
    """Signup request schema"""
    email: EmailStr
    password: str
    software_level: SoftwareLevel
    learning_goal: str
    hardware_access: HardwareAccess
    technical_comfort: TechnicalComfort
    
    @field_validator('password')
    @classmethod
    def validate_password(cls, v):
        if len(v) < 6:
            raise ValueError('Password must be at least 6 characters long')
        return v
    
    @field_validator('learning_goal')
    @classmethod
    def validate_learning_goal(cls, v):
        if not v or len(v) > 200:
            raise ValueError('Learning goal must be between 1 and 200 characters')
        # Basic XSS prevention
        if '<' in v or '>' in v or 'script' in v.lower():
            raise ValueError('Learning goal contains invalid characters')
        return v


class SignupResponse(BaseModel):
    """Signup response schema"""
    user_id: str
    email: str
    message: str
    model_config = ConfigDict(from_attributes=True)


class SigninRequest(BaseModel):
    """Signin request schema"""
    email: EmailStr
    password: str


class SigninResponse(BaseModel):
    """Signin response schema"""
    user_id: str
    email: str
    message: str
    model_config = ConfigDict(from_attributes=True)


class ProfileResponse(BaseModel):
    """User profile response schema"""
    user_id: str
    email: str
    software_level: SoftwareLevel
    learning_goal: str
    hardware_access: HardwareAccess
    technical_comfort: TechnicalComfort
    model_config = ConfigDict(from_attributes=True)


class ProfileUpdateRequest(BaseModel):
    """Profile update request schema"""
    software_level: Optional[SoftwareLevel] = None
    learning_goal: Optional[str] = None
    hardware_access: Optional[HardwareAccess] = None
    technical_comfort: Optional[TechnicalComfort] = None
    
    @field_validator('learning_goal')
    @classmethod
    def validate_learning_goal(cls, v):
        if v is not None:
            if len(v) > 200:
                raise ValueError('Learning goal must be at most 200 characters')
            if '<' in v or '>' in v or 'script' in v.lower():
                raise ValueError('Learning goal contains invalid characters')
        return v


class PasswordResetRequest(BaseModel):
    """Password reset request schema"""
    email: EmailStr


class PasswordResetConfirm(BaseModel):
    """Password reset confirmation schema"""
    token: str
    new_password: str
    
    @field_validator('new_password')
    @classmethod
    def validate_password(cls, v):
        if len(v) < 6:
            raise ValueError('Password must be at least 6 characters long')
        return v


class ErrorResponse(BaseModel):
    """Error response schema"""
    error: str
    code: str
    message: str
