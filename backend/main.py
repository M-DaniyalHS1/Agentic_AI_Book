"""
FastAPI Backend for AI-Native Digital Textbook RAG Chatbot
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os

from src.api.content_router import router as content_router
from src.api.tutor_router import router as tutor_router
from src.services.qdrant_client import init_qdrant_collection

app = FastAPI(
    title="AI Textbook RAG API",
    description="RAG-based chatbot API for Physical AI textbook",
    version="1.0.0"
)

# CORS for frontend communication - Configured for Vercel production
ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000,http://localhost:8080").split(",")
# Add Vercel preview and production domains
VERCEL_DOMAINS = [
    "https://agent-book-factory.vercel.app",
    "https://agent-book-factory-git-main.vercel.app",
]
ALLOWED_ORIGINS.extend(VERCEL_DOMAINS)

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(content_router)
app.include_router(tutor_router)


@app.on_event("startup")
async def startup_event():
    """Initialize services on startup"""
    # Initialize Qdrant collection (skip if no QDRANT_URL - serverless mode)
    qdrant_url = os.getenv("QDRANT_URL", "")
    if qdrant_url:
        try:
            init_qdrant_collection()
            print("✓ Qdrant collection initialized")
        except Exception as e:
            print(f"⚠ Qdrant initialization warning: {e}")
    else:
        print("⚠ QDRANT_URL not set - skipping Qdrant initialization (serverless mode)")


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "version": "1.0.0",
        "services": {
            "api": "running",
            "rag": "ready"
        }
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
