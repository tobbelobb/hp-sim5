#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$HP_SIM5_ROOT"

if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo 'This setup targets Linux x86_64 cloud machines.' >&2
  exit 1
fi
for tool in python3 cc c++ curl git tar sha256sum; do
  command -v "$tool" >/dev/null || { echo "Missing prerequisite: $tool" >&2; exit 1; }
done
test -x "$PUPPETEER_EXECUTABLE_PATH" || {
  echo "Install Chromium or set PUPPETEER_EXECUTABLE_PATH (currently $PUPPETEER_EXECUTABLE_PATH)." >&2
  exit 1
}
mkdir -p "$HP_SIM5_CLOUD_TOOLS/node" "$XDG_CACHE_HOME" "$MPLCONFIGDIR" \
  "$WARP_CACHE_PATH" "$PIP_CACHE_DIR" "$npm_config_cache" output/cloud

# Node 24.19.0 aborted in napi_detach_arraybuffer during October 2026 onboarding.
# Node 23 matches the repository CI major version used for that validation.
if [[ ! -x "$HP_SIM5_CLOUD_TOOLS/node/node-v$HP_SIM5_NODE_VERSION-linux-x64/bin/node" ]]; then
  (
    cd "$HP_SIM5_CLOUD_TOOLS/node"
    archive="node-v$HP_SIM5_NODE_VERSION-linux-x64.tar.xz"
    curl -fsSLO "https://nodejs.org/dist/v$HP_SIM5_NODE_VERSION/$archive"
    curl -fsSL "https://nodejs.org/dist/v$HP_SIM5_NODE_VERSION/SHASUMS256.txt" -o "SHASUMS256-$HP_SIM5_NODE_VERSION.txt"
    awk -v archive="$archive" '$2 == archive' "SHASUMS256-$HP_SIM5_NODE_VERSION.txt" | sha256sum -c -
    tar -xf "$archive"
  )
fi
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
fi
.venv/bin/python -m pip install -r requirements-warp.txt -r autocal/requirements-jax-cpu.txt cmake==4.4.4 PyYAML==6.0.3
.venv/bin/python -m pip check
npm ci

if [[ ! -x "$CARGO_HOME/bin/rustup" ]]; then
  curl -fsSL https://sh.rustup.rs -o "$HP_SIM5_CLOUD_TOOLS/rustup-init.sh"
  sh "$HP_SIM5_CLOUD_TOOLS/rustup-init.sh" -y --no-modify-path --profile minimal \
    --default-toolchain "$HP_SIM5_RUST_VERSION" --target wasm32-unknown-unknown
fi
rustup toolchain install "$HP_SIM5_RUST_VERSION" --profile minimal
rustup default "$HP_SIM5_RUST_VERSION"
rustup component add llvm-tools-preview --toolchain "$HP_SIM5_RUST_VERSION"
rustup target add wasm32-unknown-unknown

# Scope SSH-to-HTTPS rewriting to this command; reuse platform Git authentication.
git -c url.https://github.com/.insteadOf=git@github.com: \
  -c url.https://gitlab.com/.insteadOf=git@gitlab.com: submodule update --init \
  RRF/ReprapFirmware RRF/RRFLibraries RRF/CANlib autocal/auto-calibration-simulation-for-hangprinter
npm run build:native -- --no-pgo
# Preserve the tracked browser artifact during setup; compile into Cargo's cache.
cargo build --manifest-path src/wasm/cable_solver/Cargo.toml --release --target wasm32-unknown-unknown --locked
cmake -S RRF -B RRF/build
cmake --build RRF/build --target rrf_simulator -j "${HP_SIM5_BUILD_JOBS:-4}"

# Other project skills are maintained in the repository, not restored by setup.
.venv/bin/python scripts/cloud/install_playwright_skill.py
npm install --prefix "$HP_SIM5_CLOUD_TOOLS/playwright-cli" --no-audit --no-fund @playwright/cli@0.1.22
.venv/bin/python - <<'PY'
import json
import os
from pathlib import Path

config = {
    'browser': {
        'browserName': 'chromium',
        'isolated': True,
        'launchOptions': {
            'executablePath': os.environ['PUPPETEER_EXECUTABLE_PATH'],
            'headless': True,
            'args': ['--no-sandbox', '--enable-unsafe-swiftshader'],
        },
    },
    'outputDir': str(Path(os.environ['HP_SIM5_ROOT']) / 'output/playwright-cli'),
}
Path(os.environ['PLAYWRIGHT_MCP_CONFIG']).write_text(json.dumps(config, indent=2) + '\n')
PY
echo 'Setup complete. Source scripts/cloud/env.sh in each development shell.'
