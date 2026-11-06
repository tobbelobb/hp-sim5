# Combined code checks

`program.integration.IntegrationController` prepares a combined candidate from
reviewed submissions against an explicit base revision. It keeps an exclusive
lease while applying patches and checking the combined revision. Conflicts,
changed submissions and missing reviewer requirements block readiness.

`integration_check` runs explicit command arrays in the candidate worktree with a
wall deadline. The runner records commands, outputs, candidate identity and
cleanup. Physics changes retain the physics test requirement. A passing receipt
applies only to that revision and configuration. Release the lease after checks.

This controller prepares and checks a candidate; it does not merge or push it.
Final delivery uses ordinary Git under the human's integration authority.
