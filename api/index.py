"""
Vercel Serverless Function entry point for FastAPI app
Routes all API requests to the main FastAPI application
"""
from mangum import Mangum
import sys
import os

# Add backend src to path for imports
backend_dir = os.path.join(os.path.dirname(__file__), '..', 'backend')
src_dir = os.path.join(backend_dir, 'src')
sys.path.insert(0, backend_dir)
sys.path.insert(0, src_dir)

# Set environment for imports
os.environ.setdefault('PYTHONPATH', src_dir)

from main import app

# Create ASGI handler for Vercel serverless
handler = Mangum(app, lifespan="off")
