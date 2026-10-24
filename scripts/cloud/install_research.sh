#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$HP_SIM5_ROOT"
bash scripts/cloud/install.sh
.venv/bin/python scripts/cloud/install_research_client.py
