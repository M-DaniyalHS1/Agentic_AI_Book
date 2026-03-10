#!/bin/bash
# Vercel Build Script using Build Output API v3
# Creates proper .vercel/output/ structure with static files and serverless functions

set -e

echo "=== Step 1: Building Frontend ==="
cd frontend
npm run build
cd ..

echo "=== Step 2: Creating Vercel Output Structure ==="
# Remove old output directory
rm -rf .vercel/output

# Create output directories
mkdir -p .vercel/output/static
mkdir -p .vercel/output/functions/api/index.func

echo "=== Step 3: Copying Static Files ==="
# Copy frontend build to static
cp -r frontend/build/* .vercel/output/static/

echo "=== Step 4: Creating Serverless Function ==="
# Copy API function to functions directory
cp api/index.py .vercel/output/functions/api/index.func/

# Copy backend source needed for imports
cp -r api/backend .vercel/output/functions/api/index.func/

# Copy installed Python packages
cp -r api/*.py .vercel/output/functions/api/index.func/ 2>/dev/null || true
cp -r api/fastapi .vercel/output/functions/api/index.func/ 2>/dev/null || true
cp -r api/mangum .vercel/output/functions/api/index.func/ 2>/dev/null || true
cp -r api/starlette .vercel/output/functions/api/index.func/ 2>/dev/null || true
cp -r api/anyio .vercel/output/functions/api/index.func/ 2>/dev/null || true

echo "=== Step 5: Creating Routes Configuration ==="
# Create routes.json for Build Output API v3
cat > .vercel/output/routes.json <<EOF
{
  "version": 3,
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "/api/index.py"
    },
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ]
}
EOF

echo "=== Build Complete ==="
echo "Output structure:"
ls -la .vercel/output/
echo "Functions:"
ls -la .vercel/output/functions/
echo "Static files (first 10):"
ls -la .vercel/output/static/ | head -10
echo "Routes config:"
cat .vercel/output/routes.json
