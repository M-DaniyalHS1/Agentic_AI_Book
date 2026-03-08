#!/bin/bash
# Quick Deploy to Vercel Production
# This script deploys the full-stack app to Vercel

set -e

echo "======================================"
echo "  Vercel Full-Stack Deployment"
echo "======================================"
echo ""

# Check if logged in
echo "Checking Vercel login status..."
vercel whoami || {
    echo ""
    echo "❌ Not logged in to Vercel"
    echo "Run: vercel login"
    exit 1
}

echo "✓ Logged in as: $(vercel whoami)"
echo ""

# Link project (if not already linked)
if [ ! -f ".vercel/project.json" ]; then
    echo "Linking project to Vercel..."
    vercel link
else
    echo "✓ Project already linked"
fi
echo ""

# Check for environment variables
echo "⚠️  Before deploying, ensure you have set these environment variables in Vercel:"
echo "   - QDRANT_URL"
echo "   - QDRANT_API_KEY"
echo "   - OPENAI_API_KEY"
echo "   - EMBEDDING_MODEL"
echo "   - DATABASE_URL"
echo "   - ALLOWED_ORIGINS"
echo ""
read -p "Have you set the environment variables? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Please set environment variables first:"
    echo "1. Go to Vercel Dashboard"
    echo "2. Select your project"
    echo "3. Settings > Environment Variables"
    echo "4. Add variables for Production"
    echo ""
    echo "Or use CLI:"
    echo "  vercel env add QDRANT_URL production"
    echo "  vercel env add QDRANT_API_KEY production"
    echo "  ..."
    exit 1
fi

# Deploy to production
echo ""
echo "======================================"
echo "  Deploying to Production..."
echo "======================================"
echo ""

vercel --prod

echo ""
echo "======================================"
echo "  ✓ Deployment Complete!"
echo "======================================"
echo ""
echo "Next steps:"
echo "1. Visit your deployment URL"
echo "2. Test the /health endpoint"
echo "3. Initialize Qdrant collection (first request will do this)"
echo ""
echo "To view logs: vercel logs"
echo "To open dashboard: vercel --browser"
