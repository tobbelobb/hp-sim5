#!/usr/bin/env bash
set -euo pipefail

exec "$(dirname "$0")/rrf_server.sh" -c config_slideprinter.g
