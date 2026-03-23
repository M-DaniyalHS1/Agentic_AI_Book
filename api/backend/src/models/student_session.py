"""
Student Session and AI Response Models
Tracks user interactions with the AI tutor
"""
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, Float, Boolean, JSON
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..database import Base


class StudentSession(Base):
    """Student Session - Tracks a user's learning session"""
    __tablename__ = "student_sessions"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, unique=True, nullable=False, index=True)
    user_id = Column(String, nullable=True)  # Optional: for authenticated users
    started_at = Column(DateTime(timezone=True), server_default=func.now())
    last_active = Column(DateTime(timezone=True), onupdate=func.now())
    current_module = Column(String)  # Currently viewed module
    current_chapter = Column(String)  # Currently viewed chapter
    session_metadata = Column(JSON)  # Additional session metadata (renamed from 'metadata')

    # Relationships
    queries = relationship("AIQuery", back_populates="session", cascade="all, delete-orphan")


class AIQuery(Base):
    """AI Query - Stores user queries to the AI tutor"""
    __tablename__ = "ai_queries"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(Integer, ForeignKey("student_sessions.id"), nullable=False)
    query_text = Column(Text, nullable=False)
    selected_text = Column(Text)  # Text selected by user for explanation
    query_type = Column(String)  # 'general', 'selected_text', 'chapter_specific'
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    # Relationships
    session = relationship("StudentSession", back_populates="queries")
    responses = relationship("AIResponse", back_populates="query", cascade="all, delete-orphan")


class AIResponse(Base):
    """AI Response - Stores AI tutor responses with citations"""
    __tablename__ = "ai_responses"

    id = Column(Integer, primary_key=True, index=True)
    query_id = Column(Integer, ForeignKey("ai_queries.id"), nullable=False)
    answer = Column(Text, nullable=False)
    confidence_score = Column(Float)  # 0.0 to 1.0
    is_fallback = Column(Boolean, default=False)
    response_time_ms = Column(Integer)  # Response time in milliseconds
    model_used = Column(String)  # Which LLM model generated this
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    # Relationships
    query = relationship("AIQuery", back_populates="responses")
    citations = relationship("Citation", back_populates="response", cascade="all, delete-orphan")


class Citation(Base):
    """Citation - Links AI responses to textbook content"""
    __tablename__ = "citations"

    id = Column(Integer, primary_key=True, index=True)
    response_id = Column(Integer, ForeignKey("ai_responses.id"), nullable=False)
    module_slug = Column(String)
    chapter_slug = Column(String)
    section_slug = Column(String)
    content_snippet = Column(Text)  # Excerpt from the cited content
    similarity_score = Column(Float)  # Vector similarity score
    citation_order = Column(Integer, default=0)  # Order in which to display

    # Relationships
    response = relationship("AIResponse", back_populates="citations")
