# Program tools

Use the connection and program identity returned by the launcher. The controller
binds calls to that program. `capabilities` is the source for current arguments.

| Purpose | Operations |
| --- | --- |
| Inspect | overview, capabilities, prompt_portfolio, query |
| Choose work | question, approach, route_plan, route_decide, commission, assignment, decision, steering |
| Run a researcher | worker_start, worker_status, worker_answer, worker_cancel, worker_sessions |
| Check setup | runtime_check |
| Review | artifact, claim, derive, relate, submit, submission_verify, review_start, review_status, review_apply, strategy_result |
| Control execution | allocate, run_once, runtime_status, worker_usage, fund_review, pause, resume, stop_program, recover, shutdown |
| Combine reviewed code | integration_prepare, integration_check, integration_status, integration_release |

These are ordinary Python tools on a private file queue. Workers use the existing
research launcher and physics supervisor. Client adapters supply environment
setup and platform policies. Tool data and response schemas describe the current
assignment; role behavior comes only from `role-instructions/`.

The implementation still requires route, allocation and review records for
managed dispatch. Moving it has not removed those requirements. See
[strategy](strategy.md), [controller](controller.md), [review](review.md) and
[integration](integration.md).
