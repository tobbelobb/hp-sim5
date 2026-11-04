# Research communication options

See `./hp-sim5-research-program --codex --help` for accepted flags.

Both communication options default to off (`--no-grill-me --no-ask-questions`).
`--ask-questions-along-the-way` aliases `--ask-questions`; `--dont-ask` retains
the old spelling for `--no-ask-questions`. The two modes are independent:
grilling alone enables startup questions, then autonomous research.

The grill is an adaptive interview about motivation, history, expected delivery,
measurable success, constraints, budgets and the cheapest test that could show
the idea should be abandoned. It reads available history before asking questions.
Say “stop the interview” whenever you want; the manager also ends it when the
important decisions are clear. It records the brief and unresolved assumptions
before proceeding within the agreed authority. “Stop the program” still stops
the research itself. No workers are dispatched during the startup interview.

`--ask-questions` asks for consequential human decisions revealed during research,
while the manager resolves facts and routine technical choices itself. Without
it, the manager records reasonable assumptions and reports essential blockers.
Neither mode treats unanswered questions as consent. Both work with `--program`
and current-chat/cloud entry; pass them again when opening a new manager chat.
They configure the current conversation, rather than changing stored constraints.

Codex uses its installed `features.default_mode_request_user_input` control.
Claude uses its built-in [AskUserQuestion](https://code.claude.com/docs/en/tools-reference#askuserquestion-tool-behavior),
disabled with `--disallowedTools AskUserQuestion` when neither mode needs it.
The launcher injects a conversation policy into Codex developer instructions or
Claude's appended system prompt, with plain-chat questions as a fallback. Cloud
entry returns that same policy; it cannot configure the parent client's tools.

The interview approach is inspired by Matt Pocock's
[grilling skill](https://github.com/mattpocock/skills/blob/main/skills/productivity/grilling/SKILL.md):
settle prerequisite decisions before follow-ups and retrieve available facts
instead of asking the human. The repository contains its own research-specific
policy; no external skill installation is required.

Workers contact the manager through `worker_status.manager_question`; the
manager responds with `worker_answer` within the brief and selected policy. Batch workers have human question tools disabled for both clients.
Waiting uses the existing worker allocation; unanswered questions produce partial
or blocked evidence. See [worker communication](program-workers.md#clarification).

