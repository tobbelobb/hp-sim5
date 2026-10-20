#!/usr/bin/env bash
# Source this file in each shell; exported settings do not survive new tasks.
export HP_SIM5_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export HP_SIM5_CLOUD_TOOLS="${HP_SIM5_CLOUD_TOOLS:-/workspace/.tools}"
export HP_SIM5_CLOUD_CACHE="${HP_SIM5_CLOUD_CACHE:-/workspace/.cache}"
export HP_SIM5_NODE_VERSION=23.11.1
export HP_SIM5_RUST_VERSION=1.99.1

export CARGO_HOME="${CARGO_HOME:-$HP_SIM5_CLOUD_TOOLS/cargo}"
export RUSTUP_HOME="${RUSTUP_HOME:-$HP_SIM5_CLOUD_TOOLS/rustup}"
export PATH="$HP_SIM5_CLOUD_TOOLS/node/node-v$HP_SIM5_NODE_VERSION-linux-x64/bin:$CARGO_HOME/bin:$HP_SIM5_ROOT/.venv/bin:$HP_SIM5_CLOUD_TOOLS/playwright-cli/node_modules/.bin:$PATH"
export NODE_INCLUDE_DIR="$HP_SIM5_CLOUD_TOOLS/node/node-v$HP_SIM5_NODE_VERSION-linux-x64/include/node"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-$HP_SIM5_CLOUD_CACHE}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$HP_SIM5_CLOUD_CACHE/matplotlib}"
export WARP_CACHE_PATH="${WARP_CACHE_PATH:-$HP_SIM5_CLOUD_CACHE/warp}"
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$HP_SIM5_CLOUD_CACHE/pip}"
export npm_config_cache="${npm_config_cache:-$HP_SIM5_CLOUD_CACHE/npm}"
export PUPPETEER_EXECUTABLE_PATH="${PUPPETEER_EXECUTABLE_PATH:-/usr/bin/chromium}"
export PUPPETEER_SKIP_DOWNLOAD=true
export PLAYWRIGHT_MCP_CONFIG="$HP_SIM5_ROOT/output/cloud/playwright-cli.json"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
