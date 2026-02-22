# PowerShell script to check Git status and push changes

Write-Host "Checking current directory..."
Get-Location

Write-Host "Checking if Git repository exists..."
if (Test-Path ".git") {
    Write-Host "Git repository exists"
} else {
    Write-Host "Git repository does not exist, initializing..."
    git init
}

Write-Host "Configuring Git user..."
git config user.name "Qwen Assistant"
git config user.email "qwen@example.com"

Write-Host "Current branch:"
git rev-parse --abbrev-ref HEAD

Write-Host "Git status:"
git status --porcelain

Write-Host "Adding all files..."
git add .

Write-Host "Checking if there are changes to commit..."
$diffResult = git diff --cached --quiet
if ($LASTEXITCODE -ne 0) {
    Write-Host "Committing changes..."
    git commit -m "feat(git-agent): add autonomous Git workflow agent with tests and PHR"
    
    Write-Host "Pushing to remote repository..."
    git push -u origin $(git rev-parse --abbrev-ref HEAD)
} else {
    Write-Host "No changes to commit."
}