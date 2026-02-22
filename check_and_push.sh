#!/bin/bash
# Script to check Git status and push changes

echo "Checking current directory..."
pwd

echo "Checking if Git repository exists..."
if [ -d ".git" ]; then
  echo "Git repository exists"
else
  echo "Git repository does not exist, initializing..."
  git init
fi

echo "Configuring Git user..."
git config user.name "Qwen Assistant"
git config user.email "qwen@example.com"

echo "Current branch:"
git rev-parse --abbrev-ref HEAD

echo "Git status:"
git status --porcelain

echo "Adding all files..."
git add .

echo "Checking if there are changes to commit..."
if ! git diff --cached --quiet; then
  echo "Committing changes..."
  git commit -m "feat(git-agent): add autonomous Git workflow agent with tests and PHR"
  
  echo "Pushing to remote repository..."
  git push -u origin $(git rev-parse --abbrev-ref HEAD)
else
  echo "No changes to commit."
fi