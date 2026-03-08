"""
AI Tutor API Router - Endpoints for chat and tutoring
"""
from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from sqlalchemy.orm import Session
from typing import Optional
import uuid
import time

from ..database import get_db
from ..models.student_session import StudentSession, AIQuery, AIResponse, Citation
from ..services.rag_service import get_rag_service
from .schemas import ChatRequest, ChatResponse

router = APIRouter(prefix="/api/tutor", tags=["ai-tutor"])


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    """
    Chat with the AI tutor about textbook content

    The AI tutor answers questions based solely on textbook content,
    with citations to specific chapters and sections.
    """
    start_time = time.time()
    rag_service = get_rag_service()

    # Get or create session
    session_id = request.session_id
    if not session_id:
        session_id = str(uuid.uuid4())

    # Get or create session in database
    session = db.query(StudentSession).filter(
        StudentSession.session_id == session_id
    ).first()

    if not session:
        session = StudentSession(
            session_id=session_id,
            current_module=request.module,
            current_chapter=request.chapter
        )
        db.add(session)
        db.commit()
        db.refresh(session)
    else:
        # Update session activity
        session.last_active = time.time()
        if request.module:
            session.current_module = request.module
        if request.chapter:
            session.current_chapter = request.chapter
        db.commit()

    # Process query through RAG (async)
    result = await rag_service.process_query(
        query=request.query,
        selected_text=request.selected_text,
        module_filter=request.module,
        chapter_filter=request.chapter
    )

    # Save query to database
    query = AIQuery(
        session_id=session.id,
        query_text=request.query,
        selected_text=request.selected_text,
        query_type="selected_text" if request.selected_text else "general"
    )
    db.add(query)
    db.commit()
    db.refresh(query)

    # Save response to database
    response = AIResponse(
        query_id=query.id,
        answer=result["answer"],
        confidence_score=result["confidence"],
        is_fallback=result["is_fallback"],
        response_time_ms=result["response_time_ms"],
        model_used="gpt-3.5-turbo"  # Could be dynamic
    )
    db.add(response)
    db.commit()
    db.refresh(response)

    # Save citations
    for i, citation_data in enumerate(result["citations"]):
        citation = Citation(
            response_id=response.id,
            module_slug=citation_data["module_slug"],
            chapter_slug=citation_data["chapter_slug"],
            section_slug=citation_data["section_slug"],
            content_snippet=citation_data["content_snippet"],
            similarity_score=citation_data["similarity_score"],
            citation_order=i
        )
        db.add(citation)

    db.commit()

    # Format citations for response
    citations = [
        {
            "module_slug": c.module_slug,
            "chapter_slug": c.chapter_slug,
            "section_slug": c.section_slug,
            "section_title": c.section_slug.replace("-", " ").title(),
            "content_snippet": c.content_snippet,
            "similarity_score": c.similarity_score
        }
        for c in response.citations
    ]

    return ChatResponse(
        answer=result["answer"],
        citations=citations,
        confidence=result["confidence"],
        fallback=result["is_fallback"],
        session_id=session_id,
        response_time_ms=result["response_time_ms"]
    )


@router.post("/explain-selected")
async def explain_selected(
    request: ChatRequest,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db)
):
    """
    Explain selected text from the textbook

    This is a specialized endpoint for the "explain selected text" feature
    """
    if not request.selected_text:
        raise HTTPException(
            status_code=400,
            detail="selected_text is required for this endpoint"
        )

    # Reuse chat endpoint logic
    return await chat(request, background_tasks, db)


@router.get("/session/{session_id}")
async def get_session(session_id: str, db: Session = Depends(get_db)):
    """Get session history"""
    session = db.query(StudentSession).filter(
        StudentSession.session_id == session_id
    ).first()

    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    queries = db.query(AIQuery).filter(
        AIQuery.session_id == session.id
    ).order_by(AIQuery.created_at.desc()).limit(50).all()

    history = []
    for query in queries:
        responses = db.query(AIResponse).filter(
            AIResponse.query_id == query.id
        ).all()

        history.append({
            "query": {
                "text": query.query_text,
                "type": query.query_type,
                "created_at": query.created_at
            },
            "responses": [
                {
                    "answer": r.answer,
                    "confidence": r.confidence_score,
                    "is_fallback": r.is_fallback,
                    "created_at": r.created_at
                }
                for r in responses
            ]
        })

    return {
        "session": {
            "id": session.session_id,
            "started_at": session.started_at,
            "current_module": session.current_module,
            "current_chapter": session.current_chapter
        },
        "history": history
    }
