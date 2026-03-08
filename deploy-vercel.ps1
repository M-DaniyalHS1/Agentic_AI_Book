# Quick Deploy to Vercel Production
# PowerShell script for Windows

Write-Host "======================================" -ForegroundColor Cyan
Write-Host "  Vercel Full-Stack Deployment" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

# Check if logged in
Write-Host "Checking Vercel login status..."
$whoami = vercel whoami 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "❌ Not logged in to Vercel" -ForegroundColor Red
    Write-Host "Run: vercel login"
    exit 1
}

Write-Host "✓ Logged in as: $whoami" -ForegroundColor Green
Write-Host ""

# Link project (if not already linked)
if (-not (Test-Path ".vercel/project.json")) {
    Write-Host "Linking project to Vercel..." -ForegroundColor Yellow
    vercel link
} else {
    Write-Host "✓ Project already linked" -ForegroundColor Green
}
Write-Host ""

# Check for environment variables reminder
Write-Host "⚠️  Before deploying, ensure you have set these environment variables in Vercel:" -ForegroundColor Yellow
Write-Host "   - QDRANT_URL"
Write-Host "   - QDRANT_API_KEY"
Write-Host "   - OPENAI_API_KEY"
Write-Host "   - EMBEDDING_MODEL"
Write-Host "   - DATABASE_URL"
Write-Host "   - ALLOWED_ORIGINS"
Write-Host ""
$response = Read-Host "Have you set the environment variables? (y/n)"
if ($response -ne 'y' -and $response -ne 'Y') {
    Write-Host ""
    Write-Host "Please set environment variables first:" -ForegroundColor Yellow
    Write-Host "1. Go to Vercel Dashboard"
    Write-Host "2. Select your project"
    Write-Host "3. Settings > Environment Variables"
    Write-Host "4. Add variables for Production"
    Write-Host ""
    Write-Host "Or use CLI:"
    Write-Host "  vercel env add QDRANT_URL production"
    Write-Host "  vercel env add QDRANT_API_KEY production"
    Write-Host "  ..."
    exit 1
}

# Deploy to production
Write-Host ""
Write-Host "======================================" -ForegroundColor Cyan
Write-Host "  Deploying to Production..." -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

vercel --prod

Write-Host ""
Write-Host "======================================" -ForegroundColor Green
Write-Host "  ✓ Deployment Complete!" -ForegroundColor Green
Write-Host "======================================" -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Visit your deployment URL"
Write-Host "2. Test the /health endpoint"
Write-Host "3. Initialize Qdrant collection (first request will do this)"
Write-Host ""
Write-Host "To view logs: vercel logs"
Write-Host "To open dashboard: vercel --browser"
