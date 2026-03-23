"""
Vercel Serverless Function entry point for FastAPI app
This file is kept for backwards compatibility.
Use api/index.py for Vercel deployments.
"""
from mangum import Mangum
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

from main import app

# Create ASGI handler for Vercel serverless
handler = Mangum(app, lifespan="off")
