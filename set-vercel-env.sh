#!/bin/bash
# Set environment variables in Vercel for production deployment
# Run this after: vercel link

echo "Setting Vercel environment variables..."
echo ""

# Groq
echo "Enter your GROQ_API_KEY (from console.groq.com):"
read -s GROQ_KEY
echo ""
vercel env add GROQ_API_KEY production <<< "$GROQ_KEY"

# HuggingFace
echo "Enter your HUGGINGFACE_API_KEY (from huggingface.co/settings/tokens):"
read -s HF_KEY
echo ""
vercel env add HUGGINGFACE_API_KEY production <<< "$HF_KEY"

# Qdrant
echo "Enter your QDRANT_URL (from cloud.qdrant.io):"
read QDRANT_URL
vercel env add QDRANT_URL production <<< "$QDRANT_URL"

echo "Enter your QDRANT_API_KEY:"
read -s QDRANT_KEY
echo ""
vercel env add QDRANT_API_KEY production <<< "$QDRANT_KEY"

# Database
echo "Enter your DATABASE_URL (from neon.tech):"
read DB_URL
vercel env add DATABASE_URL production <<< "$DB_URL"

# CORS
vercel env add ALLOWED_ORIGINS production <<< "http://localhost:3000,https://your-app.vercel.app"

echo ""
echo "✅ All environment variables set!"
echo "Now run: vercel --prod"
