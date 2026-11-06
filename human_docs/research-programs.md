# Research programs

Give a manager a continuing research question. It delegates investigations to
`hp-sim5-research-agent`, arranges separate reviews and integrates useful changes.

```bash
./hp-sim5-research-program --codex --question 'Your question'
# Or use --claude.
./hp-sim5-research-program --codex --program PROGRAM_ID
```

Keep the returned program ID. Reopening restores program records; continue an
existing model conversation in its coding client. Add `--grill-me` to discuss
purpose, constraints and delivery authority before work starts. Add
`--ask-questions` for consequential decisions during work. Both default to off.

To manage from an existing chat, omit `--codex` and `--claude`. The response
contains the manager instructions, `program_id` and `attachment.connection`.
Use that connection path as CONNECTION:

```bash
./hp-sim5-research-program tool CONNECTION overview
./hp-sim5-research-program tool CONNECTION capabilities
./hp-sim5-research-program tool CONNECTION OPERATION --params 'JSON object'
```

`capabilities` lists the available operations and their arguments. Calls return
`ok: true` or an error; failures exit with status 2. Calls return while workers
and reviewers continue, so inspect their status in later calls. Inspect state
before retrying a timed-out change.

Opening a program starts no researcher or physics world. The controller stays
alive across CLI calls. Ask the manager to continue, change direction, explain a
finding, pause new dispatch or stop. Pause leaves existing jobs running; stop
requests cancellation. Poll status until cleanup finishes. Closing a manager
chat alone leaves the controller running.

Program investigations use `research/sessions/`; manager notes use
`research/programs/PROGRAM_ID/manager/`. Keep reports, experiments and scratch
notes together in their assigned session.
Link those results from a short program summary under `research/programs/`.
Use ordinary Git to include useful code and evidence in one reviewed, tested
squash on main, within the authority you give the manager. Program stores live
in ignored `.research-programs*/` directories; `export PROGRAM_ID` can produce a
JSON snapshot if needed. There is no special save or commit command.

[Feature structure](../research-program-feature/README.md) and
[tool reference](../research-program-feature/reference/tools.md).
