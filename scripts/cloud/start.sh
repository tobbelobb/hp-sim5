#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$HP_SIM5_ROOT"

case "${1:-vite}" in
  vite)
    exec node_modules/.bin/vite --host 127.0.0.1 --strictPort --port "${HP_SIM5_VITE_PORT:-5173}"
    ;;
  rrf)
    exec RRF/build/rrf_simulator --vsd "$HP_SIM5_ROOT/RRF/run/vsd" \
      -c "$HP_SIM5_ROOT/RRF/run/vsd/sys/config_slideprinter.g" \
      --server -p "${HP_SIM5_RRF_PORT:-8080}"
    ;;
  *)
    echo 'Usage: bash scripts/cloud/start.sh [vite|rrf]' >&2
    exit 2
    ;;
esac
