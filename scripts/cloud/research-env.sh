#!/usr/bin/env bash
# Opt in to authenticated cloud subagents as research clients.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
export HP_SIM5_CLOUD_QUEUE="${HP_SIM5_CLOUD_QUEUE:-$HP_SIM5_ROOT/output/cloud-research/client-queue}"
export PATH="$HP_SIM5_CLOUD_TOOLS/cloud-research-client/bin:$PATH"
