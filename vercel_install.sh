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
# Copy backend source code to api/backend/ for imports during runtime
# The api/index.py will import from api/backend/main.py
mkdir -p api/backend
cp -r backend/*.py api/backend/ 2>/dev/null || true
cp -r backend/src api/backend/ 2>/dev/null || true
cp -r backend/api api/backend/ 2>/dev/null || true

echo "=== Install Complete ==="
