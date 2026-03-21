"""
FastAPI Backend for AI-Native Digital Textbook RAG Chatbot
"""
from fastapi import FastAPI, Request, Response
import os

from src.api.content_router import router as content_router
from src.api.tutor_router import router as tutor_router
from src.api.auth import router as auth_router
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
# Ensure localhost variants are included
if "http://localhost:3000" not in ALLOWED_ORIGINS:
    ALLOWED_ORIGINS.append("http://localhost:3000")
if "http://127.0.0.1:3000" not in ALLOWED_ORIGINS:
    ALLOWED_ORIGINS.append("http://127.0.0.1:3000")

# Custom CORS middleware to properly handle Set-Cookie
# Using only custom middleware (not CORSMiddleware) to avoid conflicts with cookie handling
@app.middleware("http")
async def add_cors_headers(request: Request, call_next):
    origin = request.headers.get("origin", "*")
    is_allowed = origin in ALLOWED_ORIGINS or origin == "*"

    # Handle preflight OPTIONS requests
    if request.method == "OPTIONS":
        if is_allowed:
            response = Response(status_code=200)
            response.headers["Access-Control-Allow-Credentials"] = "true"
            response.headers["Access-Control-Allow-Origin"] = origin
            response.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
            response.headers["Access-Control-Expose-Headers"] = "Set-Cookie"
            return response
        else:
            return Response(status_code=403)

    # Handle regular requests
    response = await call_next(request)
    if is_allowed:
        response.headers["Access-Control-Allow-Credentials"] = "true"
        response.headers["Access-Control-Allow-Origin"] = origin
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
        response.headers["Access-Control-Expose-Headers"] = "Set-Cookie"

    return response

# Include routers
app.include_router(content_router)
app.include_router(tutor_router)
app.include_router(auth_router)


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
    
    print("✓ Database connection ready (SQLAlchemy)")


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
    # Use 'localhost' (not '0.0.0.0' or '127.0.0.1') for consistent cookie domain handling
    uvicorn.run(app, host="localhost", port=8001)
