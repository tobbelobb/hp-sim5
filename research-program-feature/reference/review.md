# Review protocol

`program.review` captures a fixed submission from an ended worker, then starts a
separate coding-client conversation. The role is loaded directly from
`role-instructions/REVIEWER.md`; generated prompt data supplies the exact target,
response schema and runtime limits.

`submit` takes a claim, base revision, optional committed candidate revision,
configuration, input identities and selected files. Files are limited to 32 files
and 32 MiB. The manifest and files have checksums. `submission_verify` rejects
changed bytes; another candidate or evidence set requires a new submission.

The reviewer returns a finding, keep/revise/block verdict, practical value, code
readiness, checks, limitations and discrepancies. Checks identify inspected,
recomputed, reproduced or tested evidence. Version 2 also supports required check
commands for final integration. An exited reviewer process alone admits nothing.
`review_apply` verifies the target again and records the assessment. Missing code
checks can block delivery while preserving a useful finding. Historical version-1
submissions retain their original interpretation.

Physics changes require a passing `npm run test:physics` receipt for the candidate,
with output inspected by the reviewer. Reports can be useful without a code patch.
Only selected files are copied; credentials and runtime connection files are
rejected. These are integrity checks within one OS user, not adversarial isolation.
