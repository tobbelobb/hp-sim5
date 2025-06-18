#!/bin/bash
set -e

rm -rf dist
npx vite build
git worktree add ../deploy-flipper-temp deploy_flipper
cd ../deploy-flipper-temp
rm -rf *
cp -r ../hp-sim5/dist/* .
touch .nojekyll
git add .
git commit -m "Deploy latest Vite build"
git push
cd ..
git worktree remove ../deploy-flipper-temp
git checkout main
