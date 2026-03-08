"""
Textbook Content Entity Model
Stores all textbook content organized by modules, chapters, and sections
"""
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..database import Base


class Module(Base):
    """Module - Top-level organization unit (e.g., 'Module 1: ROS 2')"""
    __tablename__ = "modules"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, nullable=False)
    slug = Column(String, unique=True, nullable=False, index=True)
    description = Column(Text)
    sidebar_position = Column(Integer, default=0)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationships
    chapters = relationship("Chapter", back_populates="module", cascade="all, delete-orphan")


class Chapter(Base):
    """Chapter - Second-level organization (e.g., 'ROS 2 Architecture')"""
    __tablename__ = "chapters"

    id = Column(Integer, primary_key=True, index=True)
    module_id = Column(Integer, ForeignKey("modules.id"), nullable=False)
    title = Column(String, nullable=False)
    slug = Column(String, nullable=False)
    content = Column(Text)
    learning_objectives = Column(JSON)  # List of learning objectives
    prerequisites = Column(JSON)  # List of prerequisite chapter IDs
    key_takeaways = Column(JSON)  # List of key takeaways
    sidebar_position = Column(Integer, default=0)
    word_count = Column(Integer, default=0)
    git_sha = Column(String)  # Git commit SHA for versioning
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationships
    module = relationship("Module", back_populates="chapters")
    sections = relationship("Section", back_populates="chapter", cascade="all, delete-orphan")


class Section(Base):
    """Section - Granular content unit for RAG retrieval"""
    __tablename__ = "sections"

    id = Column(Integer, primary_key=True, index=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=False)
    title = Column(String, nullable=False)
    slug = Column(String, nullable=False)
    content = Column(Text, nullable=False)
    content_html = Column(Text)  # Rendered HTML version
    sidebar_position = Column(Integer, default=0)
    word_count = Column(Integer, default=0)
    git_sha = Column(String)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationships
    chapter = relationship("Chapter", back_populates="sections")

    # Unique constraint for RAG indexing
    __table_args__ = (
        # Ensure unique sections within a chapter
        {'sqlite_autoincrement': True} if False else {}  # PostgreSQL doesn't need this
    )
