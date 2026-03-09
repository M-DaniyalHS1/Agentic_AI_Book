#!/bin/bash
# Vercel Install Script
# Installs both frontend and backend dependencies

set -e

echo "=== Installing Frontend Dependencies ==="
cd frontend
npm install
cd ..

echo "=== Installing Backend Python Dependencies ==="
# Install Python dependencies for serverless functions
# Target api/ directory for Vercel serverless functions
pip install -r backend/requirements-serverless.txt --target=./api

echo "=== Copying Backend Source for Imports ==="
# Copy backend source code to api/ for imports during runtime
mkdir -p api/backend
cp -r backend/*.py api/backend/ 2>/dev/null || true
cp -r backend/src api/backend/ 2>/dev/null || true
cp -r backend/api api/backend/ 2>/dev/null || true

# Also copy to frontend/build/api for Vercel deployment
mkdir -p frontend/build/api
cp -r api/*.py frontend/build/api/ 2>/dev/null || true
cp -r api/backend frontend/build/api/ 2>/dev/null || true
# Copy installed dependencies to build output
cp -r api/fastapi frontend/build/api/ 2>/dev/null || true
cp -r api/mangum* frontend/build/api/ 2>/dev/null || true
cp -r api/uvicorn* frontend/build/api/ 2>/dev/null || true
cp -r api/pydantic* frontend/build/api/ 2>/dev/null || true
cp -r api/starlette* frontend/build/api/ 2>/dev/null || true
cp -r api/anyio* frontend/build/api/ 2>/dev/null || true
cp -r api/sniffio* frontend/build/api/ 2>/dev/null || true
cp -r api/typing_extensions* frontend/build/api/ 2>/dev/null || true
cp -r api/httpx* frontend/build/api/ 2>/dev/null || true
cp -r api/httpcore* frontend/build/api/ 2>/dev/null || true
cp -r api/h11* frontend/build/api/ 2>/dev/null || true
cp -r api/certifi* frontend/build/api/ 2>/dev/null || true
cp -r api/idna* frontend/build/api/ 2>/dev/null || true

echo "=== Install Complete ==="
