"""
Content API Router - Endpoints for accessing textbook content
"""
from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.orm import Session
from typing import List, Optional
import os
import markdown

from ..database import get_db
from ..models.textbook_content import Module, Chapter, Section

router = APIRouter(prefix="/api/content", tags=["content"])


def markdown_to_html(content: str) -> str:
    """Convert markdown to HTML"""
    return markdown.markdown(
        content,
        extensions=['extra', 'codehilite', 'toc']
    )


@router.get("/modules")
async def get_modules(db: Session = Depends(get_db)):
    """Get all modules"""
    modules = db.query(Module).order_by(Module.sidebar_position).all()
    return {
        "modules": [
            {
                "id": m.id,
                "title": m.title,
                "slug": m.slug,
                "description": m.description,
                "chapter_count": len(m.chapters)
            }
            for m in modules
        ]
    }


@router.get("/modules/{module_slug}")
async def get_module(module_slug: str, db: Session = Depends(get_db)):
    """Get a specific module with its chapters"""
    module = db.query(Module).filter(Module.slug == module_slug).first()
    if not module:
        raise HTTPException(status_code=404, detail="Module not found")

    chapters = [
        {
            "id": c.id,
            "title": c.title,
            "slug": c.slug,
            "sidebar_position": c.sidebar_position,
            "section_count": len(c.sections)
        }
        for c in sorted(module.chapters, key=lambda x: x.sidebar_position)
    ]

    return {
        "module": {
            "id": module.id,
            "title": module.title,
            "slug": module.slug,
            "description": module.description,
            "chapters": chapters
        }
    }


@router.get("/chapters/{chapter_slug}")
async def get_chapter(chapter_slug: str, db: Session = Depends(get_db)):
    """Get a specific chapter with its sections"""
    chapter = db.query(Chapter).filter(Chapter.slug == chapter_slug).first()
    if not chapter:
        raise HTTPException(status_code=404, detail="Chapter not found")

    module = chapter.module

    sections = [
        {
            "id": s.id,
            "title": s.title,
            "slug": s.slug,
            "sidebar_position": s.sidebar_position,
            "word_count": s.word_count
        }
        for s in sorted(chapter.sections, key=lambda x: x.sidebar_position)
    ]

    return {
        "chapter": {
            "id": chapter.id,
            "title": chapter.title,
            "slug": chapter.slug,
            "content": chapter.content,
            "content_html": markdown_to_html(chapter.content) if chapter.content else None,
            "learning_objectives": chapter.learning_objectives,
            "prerequisites": chapter.prerequisites,
            "key_takeaways": chapter.key_takeaways,
            "module": {
                "slug": module.slug,
                "title": module.title
            },
            "sections": sections
        }
    }


@router.get("/sections/{section_slug}")
async def get_section(section_slug: str, db: Session = Depends(get_db)):
    """Get a specific section with full content"""
    section = db.query(Section).filter(Section.slug == section_slug).first()
    if not section:
        raise HTTPException(status_code=404, detail="Section not found")

    chapter = section.chapter
    module = chapter.module

    # Get navigation (prev/next sections)
    sections_in_chapter = sorted(chapter.sections, key=lambda x: x.sidebar_position)
    current_idx = next((i for i, s in enumerate(sections_in_chapter) if s.id == section.id), -1)

    prev_section = None
    next_section = None

    if current_idx > 0:
        prev = sections_in_chapter[current_idx - 1]
        prev_section = {
            "slug": prev.slug,
            "title": prev.title
        }

    if current_idx < len(sections_in_chapter) - 1:
        next_s = sections_in_chapter[current_idx + 1]
        next_section = {
            "slug": next_s.slug,
            "title": next_s.title
        }

    return {
        "section": {
            "id": section.id,
            "title": section.title,
            "slug": section.slug,
            "content": section.content,
            "content_html": markdown_to_html(section.content),
            "word_count": section.word_count,
            "chapter": {
                "slug": chapter.slug,
                "title": chapter.title
            },
            "module": {
                "slug": module.slug,
                "title": module.title
            },
            "navigation": {
                "prev": prev_section,
                "next": next_section
            }
        }
    }


@router.get("/search")
async def search_content(
    q: str,
    limit: int = 10,
    module_filter: Optional[str] = None,
    db: Session = Depends(get_db)
):
    """
    Search textbook content (full-text search)
    """
    # Simple full-text search using SQL LIKE
    # For production, use PostgreSQL full-text search or Elasticsearch
    search_term = f"%{q}%"

    query = db.query(Section).filter(
        Section.content.ilike(search_term) | Section.title.ilike(search_term)
    )

    if module_filter:
        query = query.join(Chapter).join(Module).filter(
            Module.slug == module_filter
        )

    results = query.limit(limit).all()

    formatted_results = []
    for section in results:
        # Find matching snippet
        content_lower = section.content.lower()
        q_lower = q.lower()
        idx = content_lower.find(q_lower)

        if idx >= 0:
            start = max(0, idx - 100)
            end = min(len(section.content), idx + len(q) + 100)
            snippet = "..." + section.content[start:end] + "..."
        else:
            snippet = section.content[:200] + "..."

        formatted_results.append({
            "id": section.id,
            "title": section.title,
            "slug": section.slug,
            "snippet": snippet,
            "chapter_slug": section.chapter.slug,
            "chapter_title": section.chapter.title,
            "module_slug": section.chapter.module.slug,
            "module_title": section.chapter.module.title
        })

    return {
        "query": q,
        "results": formatted_results,
        "total": len(results)
    }
