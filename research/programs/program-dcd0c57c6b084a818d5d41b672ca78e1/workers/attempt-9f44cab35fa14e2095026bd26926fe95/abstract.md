# Hangprinter research startup failure: Localhost socket creation denied during startup

Status: blocked before scientific work. Manager-authored setup record.

## Abstract

The native rebuild succeeded. The launcher then received Operation not permitted while creating a localhost socket. No actor, numerical experiment or simulation movement ran. The launcher recorded return code 1 and returned through its cleanup path. This is setup evidence, with no calibration or physics finding. A distinct replacement allocation corrected the documented runtime environment and command networking.

## Topics

Hangprinter, research launcher, cloud setup, failure preservation

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The numerical research question | None | Not tested | [Exit observation](exit.json) |

## Decision handoff

- Scope: startup of this attempt only.
- Review: manager inspected launcher logs and exit observation.
- Decision: preserve the failure; no scientific inference.
- Next: continue under the corrected allocation in the same program.

## References

- [Notebook](research.md): original objective and manager setup note.
- [Report](report.md): failure boundary.
