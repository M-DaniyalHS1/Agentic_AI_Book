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
pip install -r backend/requirements-serverless.txt --target=./api

echo "=== Install Complete ==="
