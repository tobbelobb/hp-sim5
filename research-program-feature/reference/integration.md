# Reviewed delivery

`integration_prepare` combines reviewed submissions against an explicit base.
Conflicts, changed evidence and reviewer requirements prevent readiness.
`integration_check` runs relevant command arrays on the clean combined candidate.
Receipts identify its tree, inputs, configuration, check output and cleanup.
Physics changes require `npm run test:physics`.

A prepared or checked candidate leaves the program active. `integration_finish`
requires passing checks, current accepted reviews, unchanged evidence, the
program's summary.html in the candidate and main still at the chosen base. It
creates one commit with that exact checked tree and main as its sole parent,
fast-forwards a clean main worktree (or updates main if it is not checked out),
and marks the program completed. It never pushes. Useful findings without code
improvements can be delivered through the same path as report/evidence changes.

Include original session reports and useful evidence in the reviewed candidate,
along with the short program summary. Do not publish copies from the private
submission snapshots. A changed main requires preparing and checking again.
