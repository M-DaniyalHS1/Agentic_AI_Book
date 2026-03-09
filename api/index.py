"""
Vercel Serverless Function entry point for FastAPI app
Routes all API requests to the main FastAPI application
"""
from mangum import Mangum
import sys
import os

# Get the directory where this file is located
current_dir = os.path.dirname(os.path.abspath(__file__))

# In Vercel serverless, the structure is:
# /api/index.py (this file)
# /backend/src/...
# Try multiple path strategies
path_strategies = [
    # Strategy 1: api/ is at root level
    os.path.join(current_dir, '..', 'backend'),
    os.path.join(current_dir, '..', 'backend', 'src'),
    # Strategy 2: api/ is inside backend/
    os.path.join(current_dir, 'backend'),
    os.path.join(current_dir, 'backend', 'src'),
    # Strategy 3: Flat structure
    current_dir,
    os.path.join(current_dir, 'src'),
]

# Add all valid paths to sys.path
for path in path_strategies:
    if os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)

# Also add the parent directory
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# Set environment for imports
os.environ.setdefault('PYTHONPATH', ':'.join(sys.path))

# Debug logging (helps troubleshoot path issues)
print(f"=== Python Path Setup ===")
print(f"Current dir: {current_dir}")
print(f"Parent dir: {parent_dir}")
print(f"Sys path: {sys.path[:5]}...")  # First 5 entries

# Import and create handler
try:
    from main import app
    print("✓ Successfully imported main.app")
except ImportError as e:
    print(f"✗ Import error: {e}")
    print(f"Available paths: {sys.path}")
    # Try alternative import
    try:
        from backend.main import app
        print("✓ Successfully imported backend.main.app (fallback)")
    except ImportError as e2:
        print(f"✗ Fallback import also failed: {e2}")
        raise

# Create ASGI handler for Vercel serverless
handler = Mangum(app, lifespan="off")
print("✓ Mangum handler created successfully")
