"""
Vercel Serverless Function entry point for FastAPI app
Routes all API requests to the main FastAPI application
"""
from mangum import Mangum
import sys
import os

# Get the directory where this file is located
current_dir = os.path.dirname(os.path.abspath(__file__))
# Root directory is one level up
root_dir = os.path.dirname(current_dir)
backend_dir = os.path.join(root_dir, 'backend')
src_dir = os.path.join(backend_dir, 'src')

# Add to Python path
sys.path.insert(0, root_dir)
sys.path.insert(0, backend_dir)
sys.path.insert(0, src_dir)

# Set environment for imports
os.environ.setdefault('PYTHONPATH', src_dir)

# Import and create handler
from main import app

# Create ASGI handler for Vercel serverless
handler = Mangum(app, lifespan="off")
