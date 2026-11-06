# Program tools

Use the connection returned by the launcher. `capabilities` describes current
arguments. The ordinary path is:

1. `overview` for short status; `commission` with a question and settings.
2. `worker_start`, then `worker_status`; use `worker_answer` if needed.
3. `submit` selecting the canonical report/evidence and committed candidate.
4. `review_start`, `review_status`, `review_apply`.
5. `integration_prepare`, `integration_check`, `integration_status`.
6. `integration_finish` with a human-readable commit message.

`commission` returns the ready assignment ID. It requires no plan, hypothesis,
experiment design or accounting. The worker machine is explicit in settings.
`submit` supplies an internal finding identity automatically; detailed evidence
graph operations remain available when useful.

`pause`, `resume`, `stop` and `shutdown` control execution; `worker_cancel` and
`review_cancel` stop individual children. `recover` acknowledges lost ownership;
`integration_recover` and `integration_release` handle candidate leases.
`runtime_check` checks setup. `configure_pool` changes worker concurrency.

State and verification snapshots stay in ignored storage. No program ledger or
copied worker report is generated into research/. See [role context](context.md),
[review](review.md), [execution](controller.md) and [integration](integration.md).
