"""
Vercel Serverless Function entry point for FastAPI app
"""
from mangum import Mangum
from main import app

# Create ASGI handler for Vercel serverless
handler = Mangum(app, lifespan="off")
