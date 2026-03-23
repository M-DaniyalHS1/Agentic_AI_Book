"""
Pydantic Schemas for API requests/responses
"""
from pydantic import BaseModel, Field
from typing import Optional, List


class ChatRequest(BaseModel):
    """Request schema for chat endpoint"""
    query: str = Field(..., description="User's question or query")
    selected_text: Optional[str] = Field(None, description="Text selected by user for explanation")
    module: Optional[str] = Field(None, description="Current module context")
    chapter: Optional[str] = Field(None, description="Current chapter context")
    session_id: Optional[str] = Field(None, description="Session ID for conversation history")


class CitationResponse(BaseModel):
    """Citation in chat response"""
    module_slug: str
    chapter_slug: str
    section_slug: str
    section_title: str
    content_snippet: str
    similarity_score: float


class ChatResponse(BaseModel):
    """Response schema for chat endpoint"""
    answer: str = Field(..., description="AI tutor's answer")
    citations: List[CitationResponse] = Field(default_factory=list, description="List of citations")
    confidence: float = Field(..., description="Confidence score (0.0-1.0)")
    fallback: bool = Field(default=False, description="Whether this is a fallback response")
    session_id: Optional[str] = Field(None, description="Session ID")
    response_time_ms: int = Field(0, description="Response time in milliseconds")


class SearchRequest(BaseModel):
    """Request schema for search endpoint"""
    query: str
    module_filter: Optional[str] = None
    limit: int = 10


class SearchResult(BaseModel):
    """Search result item"""
    id: int
    title: str
    slug: str
    snippet: str
    chapter_slug: str
    chapter_title: str
    module_slug: str
    module_title: str


class SearchResponse(BaseModel):
    """Response schema for search endpoint"""
    query: str
    results: List[SearchResult]
    total: int


class ModuleInfo(BaseModel):
    """Module information"""
    id: int
    title: str
    slug: str
    description: Optional[str]
    chapter_count: int


class ChapterInfo(BaseModel):
    """Chapter information"""
    id: int
    title: str
    slug: str
    sidebar_position: int
    section_count: int


class SectionInfo(BaseModel):
    """Section information"""
    id: int
    title: str
    slug: str
    sidebar_position: int
    word_count: int


class NavigationInfo(BaseModel):
    """Navigation information (prev/next)"""
    prev: Optional[dict]
    next: Optional[dict]
