#!/bin/bash

echo "🛠️ Building the project..."
npm run build

echo "🚀 Deploying to gh-pages branch..."
cd dist || { echo "Error: 'dist' directory not found"; exit 1; }

# Initialize a new git repo
git init

# Remove any existing remote named 'origin'
git remote remove origin 2>/dev/null

# Add GitHub repo as remote
git remote add origin https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs.git

# Create gh-pages branch
git checkout -b gh-pages

# Add and commit all files
git add .
git commit -m "Deploy Vite site to GitHub Pages"

# Force push to gh-pages branch
git push -f origin gh-pages

cd ..

echo "✅ Deployment complete. Site should be live shortly."
