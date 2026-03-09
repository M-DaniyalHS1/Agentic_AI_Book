#!/bin/bash
# Vercel Build Script
# Builds the frontend Docusaurus site and preserves API functions

set -e

echo "=== Step 1: Backing up API directory ==="
# Copy api/ to temporary location before any operations
if [ -d "api" ]; then
    mkdir -p /tmp/vercel_api_backup
    rm -rf /tmp/vercel_api_backup/*
    cp -r api/* /tmp/vercel_api_backup/
    echo "✓ API directory backed up to /tmp/vercel_api_backup/"
    ls -la /tmp/vercel_api_backup/
else
    echo "✗ ERROR: api/ directory not found!"
    exit 1
fi

echo "=== Step 2: Building Frontend ==="
cd frontend
npm run build
cd ..

echo "=== Step 3: Restoring API to build output ==="
# Copy api/ from backup to frontend/build/api
mkdir -p frontend/build/api
rm -rf frontend/build/api/*
cp -r /tmp/vercel_api_backup/* frontend/build/api/

echo "✓ API functions copied to frontend/build/api/"
ls -la frontend/build/api/

# Verify index.py exists
if [ -f "frontend/build/api/index.py" ]; then
    echo "✓ VERIFIED: frontend/build/api/index.py exists"
else
    echo "✗ ERROR: frontend/build/api/index.py not found!"
    echo "Contents of frontend/build/api/:"
    ls -la frontend/build/api/ || true
    exit 1
fi

echo "=== Build Complete ==="
