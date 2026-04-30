#!/usr/bin/env bash
set -euo pipefail

exec "$(dirname "$0")/rrf_server.sh" -m hp3 --line-layers
