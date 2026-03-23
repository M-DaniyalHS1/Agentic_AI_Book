"""
Authentication API routes for Better Auth implementation
Handles signup, signin, signout, profile management, and password reset
"""
import os
import secrets
import bcrypt
import logging
import uuid
from datetime import datetime, timedelta, timezone
from fastapi import APIRouter, HTTPException, status, Request, Response
from fastapi.responses import JSONResponse
from jose import jwt, JWTError
from dotenv import load_dotenv
from sqlalchemy.orm import Session

from src.db.database import get_db_session
from src.models.user import User, UserProfile, Session as SessionModel
from src.auth.validations import (
    SignupRequest, SignupResponse,
    SigninRequest, SigninResponse,
    ProfileResponse, ProfileUpdateRequest,
    PasswordResetRequest, PasswordResetConfirm,
    ErrorResponse,
    SoftwareLevel, HardwareAccess, TechnicalComfort
)
from src.auth.errors import (
    AuthError, UserExistsError, InvalidCredentialsError,
    InvalidInputError, ServerError, SessionExpiredError, InvalidTokenError
)
from src.services.email import email_service

load_dotenv()

router = APIRouter(prefix="/api/auth", tags=["authentication"])

# Configuration
SECRET_KEY = os.getenv("BETTER_AUTH_SECRET", "dev-secret-key-change-in-production")
ALGORITHM = "HS256"
SESSION_EXPIRY_DAYS = int(os.getenv("SESSION_EXPIRY_DAYS", "7"))
RESET_TOKEN_EXPIRY_HOURS = 1

# Rate limiting simple implementation
rate_limit_store: dict[str, list[datetime]] = {}

logger = logging.getLogger(__name__)


def check_rate_limit(identifier: str, max_requests: int = 5, window_minutes: int = 1) -> bool:
    """
    Simple in-memory rate limiting
    
    Args:
        identifier: User identifier (email or IP)
        max_requests: Maximum requests allowed in window
        window_minutes: Time window in minutes
        
    Returns:
        True if request is allowed, False if rate limited
    """
    now = datetime.now()
    window_start = now - timedelta(minutes=window_minutes)
    
    if identifier not in rate_limit_store:
        rate_limit_store[identifier] = []
    
    # Clean old entries
    rate_limit_store[identifier] = [
        ts for ts in rate_limit_store[identifier] 
        if ts > window_start
    ]
    
    # Check limit
    if len(rate_limit_store[identifier]) >= max_requests:
        return False
    
    # Add current request
    rate_limit_store[identifier].append(now)
    return True


def create_session_token(user_id: str) -> str:
    """Create JWT session token"""
    expire = datetime.now(timezone.utc) + timedelta(days=SESSION_EXPIRY_DAYS)
    to_encode = {
        "user_id": user_id,
        "exp": expire,
        "iat": datetime.now(timezone.utc),
        "type": "session"
    }
    return jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)


def verify_session_token(token: str) -> dict | None:
    """Verify and decode JWT session token"""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        return None


async def get_current_user(request: Request) -> dict | None:
    """Get current authenticated user from session cookie"""
    token = request.cookies.get(os.getenv("SESSION_COOKIE_NAME", "better-auth.session_token"))
    if not token:
        return None

    payload = verify_session_token(token)
    if not payload:
        return None

    # Check if session exists in database
    with get_db_session() as db:
        session = db.query(SessionModel).filter(SessionModel.token == token).first()
        if not session or session.expires_at < datetime.now(timezone.utc):
            return None

    return {"user_id": payload["user_id"], "exp": payload["exp"]}


@router.post("/signup", response_model=SignupResponse, status_code=status.HTTP_201_CREATED)
async def signup(request: SignupRequest, req: Request):
    """
    Register a new user with email/password and background information

    - **email**: Valid email address (must be unique)
    - **password**: Minimum 6 characters
    - **software_level**: beginner | intermediate | advanced
    - **learning_goal**: Primary learning objective (max 200 chars)
    - **hardware_access**: none | basic | advanced
    - **technical_comfort**: low | medium | high
    """
    # Rate limiting - 5 requests per minute
    client_ip = req.client.host if req.client else "unknown"
    if not check_rate_limit(f"signup:{client_ip}", max_requests=5, window_minutes=1):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Too many signup attempts. Please try again later."
        )

    try:
        with get_db_session() as db:
            # Check if user exists (case-insensitive)
            existing_user = db.query(User).filter(User.email == request.email.lower()).first()
            if existing_user:
                logger.warning(f"Signup attempt with existing email: {request.email}")
                raise UserExistsError()

            # Hash password
            password_hash = bcrypt.hashpw(request.password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')

            # Create user with profile
            user_id = str(uuid.uuid4())
            user = User(
                id=user_id,
                email=request.email.lower(),
                password_hash=password_hash,
            )

            profile = UserProfile(
                id=str(uuid.uuid4()),
                user_id=user_id,
                software_level=request.software_level.value,
                learning_goal=request.learning_goal,
                hardware_access=request.hardware_access.value,
                technical_comfort=request.technical_comfort.value,
            )

            db.add(user)
            db.add(profile)
            db.commit()
            db.refresh(user)
            db.refresh(profile)

            # Create session
            token = create_session_token(user.id)
            expires_at = datetime.now(timezone.utc) + timedelta(days=SESSION_EXPIRY_DAYS)

            session = SessionModel(
                id=str(uuid.uuid4()),
                user_id=user_id,
                token=token,
                expires_at=expires_at,
            )
            db.add(session)
            db.commit()

            logger.info(f"User signup successful: {user.email} (ID: {user.id})")

            # Send welcome email (non-blocking, don't wait)
            try:
                await email_service.send_welcome_email(user.email)
            except Exception as e:
                logger.warning(f"Failed to send welcome email: {e}")

            # Set session cookie
            # For local development (HTTP), secure=False; for production (HTTPS), secure=True
            is_production = os.getenv("NODE_ENV") == "production" or os.getenv("VERCEL") == "1"

            response = JSONResponse(
                content={
                    "user_id": user.id,
                    "email": user.email,
                    "message": "Account created successfully"
                }
            )
            response.set_cookie(
                key=os.getenv("SESSION_COOKIE_NAME", "better-auth.session_token"),
                value=token,
                httponly=True,
                secure=is_production,  # HTTPS only in production
                samesite="lax",
                max_age=SESSION_EXPIRY_DAYS * 24 * 60 * 60,
                path="/",
                # No domain set - browser will use default (current host)
            )

            cookie_value = response.headers.get('set-cookie')
            logger.info(f"Signup cookie set: {cookie_value}")
            logger.info(f"All response headers: {dict(response.headers)}")

            return response

    except UserExistsError as e:
        logger.warning(f"UserExistsError: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": e.message, "code": e.code}
        )
    except Exception as e:
        logger.error(f"Signup error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "Internal server error", "code": "SERVER_ERROR"}
        )


@router.post("/signin", response_model=SigninResponse)
async def signin(request: SigninRequest, req: Request):
    """
    Authenticate existing user with email/password

    Returns session cookie on successful authentication
    """
    # Rate limiting - 5 requests per minute
    client_ip = req.client.host if req.client else "unknown"
    if not check_rate_limit(f"signin:{request.email.lower()}", max_requests=5, window_minutes=1):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Too many signin attempts. Please try again later."
        )

    try:
        with get_db_session() as db:
            # Find user by email (case-insensitive)
            user = db.query(User).filter(User.email == request.email.lower()).first()

            if not user:
                # Generic error to prevent email enumeration
                logger.warning(f"Signin attempt with non-existent email: {request.email}")
                raise InvalidCredentialsError()

            # Verify password
            password_valid = bcrypt.checkpw(
                request.password.encode('utf-8'),
                user.password_hash.encode('utf-8')
            )

            if not password_valid:
                logger.warning(f"Signin attempt with invalid password for: {request.email}")
                raise InvalidCredentialsError()

            # Delete old sessions
            db.query(SessionModel).filter(SessionModel.user_id == user.id).delete()

            # Create new session
            token = create_session_token(user.id)
            expires_at = datetime.now(timezone.utc) + timedelta(days=SESSION_EXPIRY_DAYS)

            session = SessionModel(
                id=str(uuid.uuid4()),
                user_id=user.id,
                token=token,
                expires_at=expires_at,
            )
            db.add(session)
            db.commit()

            logger.info(f"User signin successful: {user.email} (ID: {user.id})")

            # Set session cookie
            # For local development (HTTP), secure=False; for production (HTTPS), secure=True
            is_production = os.getenv("NODE_ENV") == "production" or os.getenv("VERCEL") == "1"

            response = JSONResponse(
                content={
                    "user_id": user.id,
                    "email": user.email,
                    "message": "Signed in successfully"
                }
            )
            response.set_cookie(
                key=os.getenv("SESSION_COOKIE_NAME", "better-auth.session_token"),
                value=token,
                httponly=True,
                secure=is_production,
                samesite="lax",
                max_age=SESSION_EXPIRY_DAYS * 24 * 60 * 60,
                path="/",
                # No domain set - browser will use default (current host)
            )
            # Explicitly expose Set-Cookie header for CORS
            response.headers["Access-Control-Expose-Headers"] = "Set-Cookie"
            response.headers["Access-Control-Allow-Credentials"] = "true"

            cookie_value = response.headers.get('set-cookie')
            logger.info(f"Signin cookie set: {cookie_value}")
            logger.info(f"Signin response headers: {dict(response.headers)}")
            logger.info(f"Origin header: {req.headers.get('origin', 'not present')}")

            return response

    except InvalidCredentialsError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": e.message, "code": e.code}
        )
    except Exception as e:
        logger.error(f"Signin error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "Internal server error", "code": "SERVER_ERROR"}
        )


@router.post("/signout")
async def signout(request: Request):
    """
    Sign out current user by invalidating session

    Requires valid session cookie
    """
    try:
        token = request.cookies.get(os.getenv("SESSION_COOKIE_NAME", "better-auth.session_token"))

        if token:
            # Delete session from database
            with get_db_session() as db:
                db.query(SessionModel).filter(SessionModel.token == token).delete()
            logger.info("User signed out successfully")

        # Clear cookie
        response = JSONResponse(
            content={"message": "Signed out successfully"}
        )
        response.delete_cookie(
            key=os.getenv("SESSION_COOKIE_NAME", "better-auth.session_token"),
            path="/",
        )

        return response

    except Exception as e:
        logger.error(f"Signout error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "Internal server error", "code": "SERVER_ERROR"}
        )


@router.get("/profile", response_model=ProfileResponse)
async def get_profile(request: Request):
    """
    Get current user's profile

    Requires valid authentication
    """
    try:
        user_info = await get_current_user(request)
        if not user_info:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"error": "Authentication required", "code": "UNAUTHORIZED"}
            )

        with get_db_session() as db:
            user = db.query(User).filter(User.id == user_info["user_id"]).first()
            
            if not user or not user.profile:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail={"error": "Profile not found", "code": "NOT_FOUND"}
                )

            return {
                "user_id": user.id,
                "email": user.email,
                "software_level": SoftwareLevel(user.profile.softwareLevel),
                "learning_goal": user.profile.learningGoal,
                "hardware_access": HardwareAccess(user.profile.hardwareAccess),
                "technical_comfort": TechnicalComfort(user.profile.technicalComfort),
            }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get profile error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "Internal server error", "code": "SERVER_ERROR"}
        )
        
@router.put("/profile", response_model=ProfileResponse)
async def update_profile(request_body: ProfileUpdateRequest, request: Request):
    """
    Update current user's profile

    All fields are optional. Only provided fields will be updated.
    Requires valid authentication
    """
    try:
        user_info = await get_current_user(request)
        if not user_info:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"error": "Authentication required", "code": "UNAUTHORIZED"}
            )

        with get_db_session() as db:
            # Build update data
            update_data = {}
            if request_body.software_level is not None:
                update_data["softwareLevel"] = request_body.software_level.value
            if request_body.learning_goal is not None:
                update_data["learningGoal"] = request_body.learning_goal
            if request_body.hardware_access is not None:
                update_data["hardwareAccess"] = request_body.hardware_access.value
            if request_body.technical_comfort is not None:
                update_data["technicalComfort"] = request_body.technical_comfort.value

            if not update_data:
                raise InvalidInputError("No fields to update")

            # Update profile
            profile = db.query(UserProfile).filter(UserProfile.user_id == user_info["user_id"]).first()
            
            if not profile:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail={"error": "Profile not found", "code": "NOT_FOUND"}
                )
            
            for key, value in update_data.items():
                setattr(profile, key, value)
            
            db.commit()
            db.refresh(profile)

            user = db.query(User).filter(User.id == user_info["user_id"]).first()

            logger.info(f"Profile updated for user: {user.email} (ID: {user.id})")

            return {
                "user_id": user.id,
                "email": user.email,
                "software_level": SoftwareLevel(profile.softwareLevel),
                "learning_goal": profile.learningGoal,
                "hardware_access": HardwareAccess(profile.hardwareAccess),
                "technical_comfort": TechnicalComfort(profile.technicalComfort),
            }

    except InvalidInputError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": e.message, "code": e.code}
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Update profile error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "Internal server error", "code": "SERVER_ERROR"}
        )


@router.post("/reset-password")
async def request_password_reset(request_data: PasswordResetRequest, req: Request):
    """
    Request password reset email

    Sends reset link to email if user exists.
    Uses generic success message to prevent email enumeration.
    """
    # Rate limiting - 3 requests per hour
    client_ip = req.client.host if req.client else "unknown"
    if not check_rate_limit(f"reset:{request_data.email.lower()}", max_requests=3, window_minutes=60):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Too many reset requests. Please try again later."
        )

    try:
        with get_db_session() as db:
            # Find user (don't reveal if email exists)
            user = db.query(User).filter(User.email == request_data.email.lower()).first()

            if user:
                # Generate reset token
                reset_token = secrets.token_urlsafe(32)
                expires_at = datetime.utcnow() + timedelta(hours=RESET_TOKEN_EXPIRY_HOURS)

                # Store reset token in session table with special prefix
                session = SessionModel(
                    id=str(uuid.uuid4()),
                    user_id=user.id,
                    token=f"reset:{reset_token}",
                    expires_at=expires_at,
                )
                db.add(session)
                db.commit()

                # Create reset link
                reset_link = f"https://agent-book-factory.vercel.app/reset-password/confirm?token={reset_token}"

                # Send email
                try:
                    await email_service.send_password_reset_email(user.email, reset_link)
                    logger.info(f"Password reset email sent to: {user.email}")
                except Exception as e:
                    logger.error(f"Failed to send reset email: {e}")

            # Always return same message (prevent enumeration)
            return {
                "message": "If an account exists with this email, a password reset link has been sent."
            }

    except Exception as e:
        logger.error(f"Password reset request error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "Internal server error", "code": "SERVER_ERROR"}
        )


@router.post("/reset-password/confirm")
async def confirm_password_reset(request_data: PasswordResetConfirm):
    """
    Confirm password reset with token

    Sets new password and invalidates all existing sessions.
    """
    try:
        with get_db_session() as db:
            # Find reset token
            reset_session = db.query(SessionModel).filter(SessionModel.token == f"reset:{request_data.token}").first()

            if not reset_session:
                raise InvalidTokenError()

            # Check expiration
            if reset_session.expires_at < datetime.utcnow():
                # Clean up expired token
                db.delete(reset_session)
                db.commit()
                raise InvalidTokenError()

            # Hash new password
            password_hash = bcrypt.hashpw(
                request_data.new_password.encode('utf-8'),
                bcrypt.gensalt()
            ).decode('utf-8')

            # Update user password
            user = db.query(User).filter(User.id == reset_session.user_id).first()
            if user:
                user.passwordHash = password_hash

            # Invalidate ALL sessions (security)
            db.query(SessionModel).filter(SessionModel.user_id == reset_session.user_id).delete()

            # Delete the reset token
            db.delete(reset_session)
            db.commit()

            logger.info(f"Password reset successful for user ID: {reset_session.user_id}")

            return {"message": "Password reset successfully. Please sign in with your new password."}

    except InvalidTokenError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": e.message, "code": e.code}
        )
    except Exception as e:
        logger.error(f"Password reset confirm error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "Internal server error", "code": "SERVER_ERROR"}
        )


@router.get("/me")
async def get_current_user_info(request: Request):
    """
    Get current authenticated user information

    Lightweight endpoint for checking authentication status
    """
    try:
        # Debug logging
        cookie_name = os.getenv("SESSION_COOKIE_NAME", "better-auth.session_token")
        cookie_value = request.cookies.get(cookie_name)
        logger.info(f"/me - Cookie '{cookie_name}' present: {cookie_value is not None}")
        logger.info(f"/me - Cookie value (truncated): {cookie_value[:20] + '...' if cookie_value and len(cookie_value) > 20 else cookie_value}")
        logger.info(f"/me - All cookies: {dict(request.cookies)}")
        
        user_info = await get_current_user(request)
        if not user_info:
            logger.info("/me - No valid user info found (unauthenticated)")
            return {"authenticated": False}

        with get_db_session() as db:
            user = db.query(User).filter(User.id == user_info["user_id"]).first()

            if not user:
                logger.info(f"/me - User not found in DB for user_id: {user_info['user_id']}")
                return {"authenticated": False}

            logger.info(f"/me - User authenticated: {user.email} (ID: {user.id})")
            return {
                "authenticated": True,
                "user_id": user.id,
                "email": user.email,
                "has_profile": user.profile is not None,
            }

    except Exception as e:
        logger.error(f"Get current user error: {e}", exc_info=True)
        return {"authenticated": False, "error": "Unable to verify authentication"}
