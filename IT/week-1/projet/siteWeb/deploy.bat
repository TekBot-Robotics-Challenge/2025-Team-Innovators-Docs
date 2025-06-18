@echo off
echo 🛠️ Building the project...
call npm run build

echo 🚀 Deploying to gh-pages branch...
cd dist

git init

git remote remove origin 2>nul
git remote add origin https://github.com/TekBot-Robotics-Challenge/2025-Team-Innovators-Docs.git

git checkout -b gh-pages

git add .

git commit -m "Deploy Vite site to GitHub Pages"

git push -f origin gh-pages

cd ..

echo ✅ Deployment complete. Site should be live shortly.
pause
