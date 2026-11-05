# Original research artifacts recovered

The original filesystem survived. Root command access failed while the agents'
execution connections could still read it. Root command access subsequently
worked again. File deletion was not observed. The cause of the connection failure
has not been established.

This directory preserves 407 original research files, 823,635,107 uncompressed
bytes, in 21 gzip archives totaling 63,526,885 bytes. It includes full command and
noise journals, datasets, fit/history artifacts, source snapshots, diagnostics,
partial recordings and failed trials. Every archived member was read back and
checked against its original SHA-256. Six live connection/control files were
deliberately excluded and are listed in `raw-artifact-manifest.json`.

The scoped program export and exact sealed 32-file submission are also committed
in the parent directory. The first conversation-only recovery branch remains a
record of the earlier rescue attempt; these original bytes supersede its missing-
artifact caveat.

## Check or restore

From the repository root:

```sh
.venv/bin/python research/programs/program-0a82df7fd4b34b8181716630492a1af4/recovery-originals/restore_session.py --verify-only
.venv/bin/python research/programs/program-0a82df7fd4b34b8181716630492a1af4/recovery-originals/restore_session.py --destination /tmp/autocal-recovered-session
```

Restoration verifies every archive and member hash, rejects unsafe paths and
refuses to overwrite differing files. Restore into a fresh directory. Runtime
connections, dependencies and active worlds are not restored. Some scripts retain
their original absolute paths; adapting or recreating those paths is separate
from recovering the evidence.

## Research outcome

No production autocal improvement was adopted from this investigation. A bounded
preload prototype was tested in an isolated harness. It preserved nominal
accuracy but did not recover the guide-friction collection failures. Three
completed calibrations stopped automatically below 5 mm on every anchor. The
guide-friction failures and torque/slack diagnostic exposed a model-validity
concern, so they cannot establish hardware reliability or impossibility.

`original-review-response.json` is the actual first reviewer's saved response.
That reviewer finished successfully, but the controller client timed out during
the execution outage before acknowledgement. The review response must not be
represented as an applied controller admission. A separately funded live review
receipt is being recovered through the normal controller workflow.

## Durability correction

For further work, preserve and push a small checkpoint after each completed
experiment, including failures. Do not defer the only remote copy until the end
of a long research turn. Large journals can be pushed as verified compressed
artifacts, while the small datasets and results remain directly inspectable.
