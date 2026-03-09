"""
Vercel Serverless Function entry point for FastAPI app
Routes all API requests to the main FastAPI application
"""
from mangum import Mangum
import sys
import os

# Get the directory where this file is located
current_dir = os.path.dirname(os.path.abspath(__file__))

# In Vercel serverless, the structure after install is:
# /api/index.py (this file)
# /api/backend/main.py
# /api/backend/src/...
# /api/fastapi/ (dependencies installed)

# Add paths to sys.path for imports
path_strategies = [
    # Strategy 1: backend/ is inside api/ (from vercel_install.sh)
    os.path.join(current_dir, 'backend'),
    os.path.join(current_dir, 'backend', 'src'),
    # Strategy 2: api/ is at root level (development)
    os.path.join(current_dir, '..', 'backend'),
    os.path.join(current_dir, '..', 'backend', 'src'),
    # Strategy 3: current directory
    current_dir,
]

# Add all valid paths to sys.path
for path in path_strategies:
    abs_path = os.path.abspath(path)
    if os.path.exists(abs_path) and abs_path not in sys.path:
        sys.path.insert(0, abs_path)

# Set PYTHONPATH environment variable
os.environ.setdefault('PYTHONPATH', ':'.join([p for p in sys.path if os.path.exists(p)]))

# Debug logging (helps troubleshoot path issues)
print("=== Python Path Setup ===")
print(f"Current dir: {current_dir}")
print(f"Sys.path entries (first 5): {sys.path[:5]}")

# Verify backend structure exists
backend_path = os.path.join(current_dir, 'backend')
main_path = os.path.join(backend_path, 'main.py')
print(f"Backend path exists: {os.path.exists(backend_path)}")
print(f"main.py exists: {os.path.exists(main_path)}")

# List files in backend directory for debugging
if os.path.exists(backend_path):
    print(f"Files in backend/: {os.listdir(backend_path)}")

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
print(f"FastAPI app title: {app.title}")
print(f"FastAPI app routes: {len(app.routes)} routes registered")
