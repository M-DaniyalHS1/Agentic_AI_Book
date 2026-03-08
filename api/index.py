"""
Vercel Serverless Function entry point for FastAPI app
Routes all API requests to the main FastAPI application
"""
from mangum import Mangum
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

from main import app

# Create ASGI handler for Vercel serverless
handler = Mangum(app, lifespan="off")
