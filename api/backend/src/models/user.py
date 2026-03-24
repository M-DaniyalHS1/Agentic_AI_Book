"""
SQLAlchemy models for authentication
"""
import uuid
from sqlalchemy import Column, String, DateTime, ForeignKey, Text
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func

from src.database import Base


class User(Base):
    """User model for authentication"""
    __tablename__ = "users"

    id = Column(String, primary_key=True)
    email = Column(String, unique=True, nullable=False, index=True)
    password_hash = Column("password_hash", String, nullable=False)
    email_verified = Column("email_verified", DateTime(timezone=True), nullable=True)
    created_at = Column("created_at", DateTime(timezone=True), server_default=func.now())
    updated_at = Column("updated_at", DateTime(timezone=True), onupdate=func.now())

    # Relationships
    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")
    sessions = relationship("Session", back_populates="user", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email})>"


class UserProfile(Base):
    """User profile with background information"""
    __tablename__ = "user_profiles"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column("user_id", String, ForeignKey("users.id", ondelete="CASCADE"), unique=True, nullable=False, index=True)
    software_level = Column("software_level", String, nullable=False)
    learning_goal = Column("learning_goal", String(200), nullable=False)
    hardware_access = Column("hardware_access", String, nullable=False)
    technical_comfort = Column("technical_comfort", String, nullable=False)
    created_at = Column("created_at", DateTime(timezone=True), server_default=func.now())
    updated_at = Column("updated_at", DateTime(timezone=True), onupdate=func.now())

    # Relationships
    user = relationship("User", back_populates="profile")

    def __repr__(self):
        return f"<UserProfile(user_id={self.user_id}, software_level={self.software_level})>"


class Session(Base):
    """Session model for authentication"""
    __tablename__ = "sessions"

    id = Column(String, primary_key=True)
    user_id = Column("user_id", String, ForeignKey("users.id", ondelete="CASCADE"), nullable=False)
    token = Column(String, unique=True, nullable=False, index=True)
    expires_at = Column("expires_at", DateTime(timezone=True), nullable=False, index=True)
    created_at = Column("created_at", DateTime(timezone=True), server_default=func.now())

    # Relationships
    user = relationship("User", back_populates="sessions")

    def __repr__(self):
        return f"<Session(id={self.id}, user_id={self.user_id}, expires_at={self.expires_at})>"
