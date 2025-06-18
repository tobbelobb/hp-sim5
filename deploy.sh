#!/bin/bash
set -e

rm -rf dist
npx vite build
git checkout deploy_flipper
git rm -rf .
cp -r dist/* .
touch .nojekyll
git add .
git commit -m "Deploy latest Vite build"
git push
git checkout main
