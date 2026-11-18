#!/bin/sh
# Run every reproducer in this directory. Exits nonzero if any defect is present.
# Pure JS stubs only: no browser, no sockets, no build, no physics.
set -u
cd "$(dirname "$0")" || exit 2
status=0
run() {
  label=$1
  shift
  printf '\n=== %s\n' "$label"
  "$@" || { printf '*** FAILED: %s\n' "$label"; status=1; }
}
run "finding 1: recorder loss blocks the protocol" node repro_recorder_blocks_protocol.mjs
run "finding 2: non-motion payloads resume" node repro_reply_resumes_pause.mjs
run "finding 3: two-client encoder failover" node --import ./ws-stub/ws-register.mjs repro_two_clients_sticky_encoder.mjs
run "latent: queue loss and Reset replay" node --import ./ws-stub/ws-register.mjs repro_queue_loss_and_reset.mjs
printf '\n=== %s\n' "$([ $status -eq 0 ] && echo 'all reproducers passed (no defect)' || echo 'defects reproduced (see above)')"
exit $status
