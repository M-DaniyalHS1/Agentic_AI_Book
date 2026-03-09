#!/bin/bash
# Vercel Build Script
# Builds the frontend Docusaurus site and preserves API functions

set -e

echo "=== Building Frontend ==="
cd frontend
npm run build
cd ..

echo "=== Copying API Functions to Build Output ==="
# Copy api/ directory to build output for serverless functions
mkdir -p frontend/build/api
cp -r api/*.py frontend/build/api/ 2>/dev/null || true
cp -r api/requirements.txt frontend/build/api/ 2>/dev/null || true

echo "=== Build Complete ==="
