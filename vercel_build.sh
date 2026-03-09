#!/bin/bash
# Vercel Build Script
# Builds the frontend Docusaurus site

set -e

echo "=== Building Frontend ==="
cd frontend
npm run build
cd ..

echo "=== Build Complete ==="
