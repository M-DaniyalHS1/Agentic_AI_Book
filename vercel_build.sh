#!/bin/bash
# Vercel Build Script
# Builds the frontend Docusaurus site and copies API to build output

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

echo "=== Step 3: Moving build output to root ==="
# Copy frontend/build contents to root build/ directory
rm -rf build/
cp -r frontend/build/ build/

echo "=== Step 4: Copying API to build output ==="
# Copy api/ from backup to build/api for Vercel serverless functions
mkdir -p build/api
cp -r /tmp/vercel_api_backup/* build/api/

echo "✓ API functions copied to build/api/"
ls -la build/api/

# Verify index.py exists
if [ -f "build/api/index.py" ]; then
    echo "✓ VERIFIED: build/api/index.py exists"
else
    echo "✗ ERROR: build/api/index.py not found!"
    echo "Contents of build/api/:"
    ls -la build/api/ || true
    exit 1
fi

echo "=== Build Complete ==="
echo "Final build structure:"
ls -la build/ | head -20
