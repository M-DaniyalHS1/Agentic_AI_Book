#!/bin/bash
# Vercel Build Script
# Builds the frontend Docusaurus site and copies to project root

set -e

echo "=== Step 1: Building Frontend ==="
cd frontend
npm run build
cd ..

echo "=== Step 2: Copying frontend build to project root ==="
# Copy frontend/build contents to project root
# This serves static files from root where Vercel expects them
rm -rf index.html docs css js assets locales favicon.ico img manifest.json
cp -r frontend/build/* ./

echo "=== Build Complete ==="
echo "Final build structure:"
ls -la | head -30
