# Set Vercel Environment Variables - PowerShell
# Run this after: vercel link

Write-Host "Setting Vercel environment variables..." -ForegroundColor Cyan
Write-Host ""

# Groq
$GROQ_KEY = Read-Host "Enter your GROQ_API_KEY (from console.groq.com)"
vercel env add GROQ_API_KEY production

# HuggingFace
$HF_KEY = Read-Host "Enter your HUGGINGFACE_API_KEY (from huggingface.co/settings/tokens)"
vercel env add HUGGINGFACE_API_KEY production

# Qdrant
$QDRANT_URL = Read-Host "Enter your QDRANT_URL (from cloud.qdrant.io)"
vercel env add QDRANT_URL production

$QDRANT_KEY = Read-Host "Enter your QDRANT_API_KEY"
vercel env add QDRANT_API_KEY production

# Database
$DB_URL = Read-Host "Enter your DATABASE_URL (from neon.tech)"
vercel env add DATABASE_URL production

# CORS
vercel env add ALLOWED_ORIGINS production

Write-Host ""
Write-Host "✓ All environment variables set!" -ForegroundColor Green
Write-Host "Now run: vercel --prod" -ForegroundColor Yellow
