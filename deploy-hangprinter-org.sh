#!/usr/bin/env bash
#
# Observe: this script assumes there's a ../hangprinter-org repo
# to copy files into and commit inside. If that repo isn't there
# then this script won't make sense.

set -euo pipefail

npx vite build --config vite.hangprinter-org.config.js

rsync -av --delete \
  --exclude='.git' \
  --exclude='.git/**' \
  --exclude='.github' \
  --exclude='.github/**' \
  --exclude='.well-known' \
  --exclude='.well-known/**' \
  --exclude='.gitlab-ci.yml' \
  --exclude='.htaccess' \
  --exclude='.gitignore' \
  --exclude='.DS_Store' \
  --exclude='robots.txt' \
  --exclude='sitemap.xml' \
  --exclude='site.webmanifest' \
  --exclude='favicon.ico' \
  --exclude='favicon-16x16.png' \
  --exclude='favicon-32x32.png' \
  --exclude='apple-touch-icon.png' \
  --exclude='android-chrome-192x192.png' \
  --exclude='android-chrome-512x512.png' \
  --exclude='safari-pinned-tab.svg' \
  --exclude='mstile-150x150.png' \
  --exclude='browserconfig.xml' \
  --exclude='LICENSE' \
  --exclude='README.md' \
  --exclude='/resources/dont_do_it.gcode' \
  --exclude='logo_orange.svg' \
  --exclude='logo_orange.svg' \
  --exclude='hangprinter4_small.jpeg' \
  dist-hangprinter-org/ ../hangprinter-org/

# Get latest commit message + short hash from hp-sim5
COMMIT_HASH=$(git rev-parse --short=6 HEAD)
COMMIT_MSG=$(git log -1 --pretty=%B)
FULL_MSG="hp-sim5 ${COMMIT_HASH}: ${COMMIT_MSG}"

# Commit in hangprinter-org repo
cd ../hangprinter-org
git add .
git commit -m "$FULL_MSG" || echo "No changes to commit."
git push
