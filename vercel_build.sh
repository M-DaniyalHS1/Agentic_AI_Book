#!/bin/bash
# Vercel Build Script
# Builds the frontend Docusaurus site and preserves API functions

set -e

echo "=== Preserving API Functions Before Build ==="
# Copy api/ directory to temporary location before frontend build cleans it
if [ -d "api" ]; then
    mkdir -p /tmp/api_backup
    cp -r api/* /tmp/api_backup/
    echo "✓ API directory backed up"
else
    echo "⚠ Warning: api/ directory not found"
fi

echo "=== Building Frontend ==="
cd frontend
npm run build
cd ..

echo "=== Restoring API Functions to Build Output ==="
# Copy api/ from backup to frontend/build/api
mkdir -p frontend/build/api
if [ -d "/tmp/api_backup" ]; then
    cp -r /tmp/api_backup/* frontend/build/api/
    echo "✓ API functions restored to frontend/build/api/"
else
    echo "✗ Error: API backup not found"
    exit 1
fi

# Verify index.py exists
if [ -f "frontend/build/api/index.py" ]; then
    echo "✓ frontend/build/api/index.py exists"
else
    echo "✗ Error: frontend/build/api/index.py not found"
    exit 1
fi

echo "=== Build Complete ==="
