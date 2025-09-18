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
