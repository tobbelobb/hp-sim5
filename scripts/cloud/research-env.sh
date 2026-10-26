#!/usr/bin/env bash
# Select authenticated cloud subagents explicitly. Never shadow the Codex CLI.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
export HP_SIM5_CLOUD_QUEUE="${HP_SIM5_CLOUD_QUEUE:-$HP_SIM5_ROOT/output/cloud-research/client-queue}"
export HP_SIM5_RESEARCH_CLIENT=cloud
