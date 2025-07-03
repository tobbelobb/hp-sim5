#!/bin/bash
set -euo pipefail
#
# --- Parse arguments ---
VERBOSE=0
for arg in "$@"; do
  if [[ "$arg" == "-v" || "$arg" == "--verbose" ]]; then
    VERBOSE=1
  fi
done

# --- Utility function to indent subcommand output (only if verbose) ---
run() {
  if [[ "$VERBOSE" == "1" ]]; then
    echo "▶ $*"
    "$@" 2>&1 | sed 's/^/    /'
  else
    "$@" > /dev/null 2>&1
  fi
}

SCRIPT_PATH="$(readlink -f "$0")"
REPO_DIR="$(dirname "$SCRIPT_PATH")"
WORKTREE_DIR="$REPO_DIR/../deploy-flipper-temp"
DEPLOY_BRANCH="deploy_flipper"
DIST_DIR="$REPO_DIR/dist"

cd "$REPO_DIR"

mkdir -p public/examples/js/slideprinter
cp examples/js/slideprinter/moveCommander.js public/examples/js/slideprinter/moveCommander.js
cp examples/js/slideprinter/kinematics.js public/examples/js/slideprinter/kinematics.js
cp examples/js/slideprinter/guessedData.js public/examples/js/slideprinter/guessedData.js

echo "🔧 Building project with Vite..."
run rm -rf dist
run npx vite build

rm public/examples/js/slideprinter/moveCommander.js
rm public/examples/js/slideprinter/kinematics.js
rm public/examples/js/slideprinter/guessedData.js

# If the worktree directory already exists, remove it first
if [ -d "${WORKTREE_DIR}" ]; then
  echo "⚠️  Worktree directory already exists at ${WORKTREE_DIR} - removing..."
  run git worktree remove --force "${WORKTREE_DIR}"
fi

echo "🌿 Adding worktree for ${DEPLOY_BRANCH}..."
run git worktree add "${WORKTREE_DIR}" "${DEPLOY_BRANCH}"

echo "🧹 Cleaning old files in ${DEPLOY_BRANCH}..."
cd "${WORKTREE_DIR}"
run rm -rf *

echo "📦 Copying build output from dist/..."
run cp -r "${DIST_DIR}"/* .

echo "📄 Adding .nojekyll to disable GitHub Jekyll processing..."
run touch .nojekyll

echo "✅ Committing and pushing changes..."
run git add .
GIT_HASH=$(git rev-parse --short HEAD)
COMMIT_MSG="Deploy Vite build from ${GIT_HASH} on $(date +'%Y-%m-%d %H:%M:%S')"
run git commit -m "${COMMIT_MSG}" || run echo "No changes to commit"
run git push

echo "🧼 Cleaning up worktree..."
cd "${REPO_DIR}"
run git worktree remove "${WORKTREE_DIR}"

echo "🚀 Deployment to branch '${DEPLOY_BRANCH}' complete!"

