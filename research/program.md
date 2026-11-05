# Research programs

A manager chat investigates a continuing question through bounded workers and
separate evidence reviews. The human supplies the question and steering.

Before commissioning work, the manager triages the registered mission prompts
in `research/prompts/` against current evidence and the unresolved decision.
See [prompt portfolio triage](program-manager.md#triage-mission-prompts).

## Start locally

```bash
./hp-sim5-research-program --codex --question 'Your question'
# Or use --claude.
./hp-sim5-research-program --codex --program PROGRAM_ID
```

Add `--grill-me` for a startup interview or `--ask-questions` for consequential
human decisions during research; both default to off. See `--codex --help` or the optional
[communication guide](program-communication.md) for details.

In cloud, say “Launch a Research Program on this question: …” to use the
[cloud skill](../.agents/skills/cloud-research-client/SKILL.md).

## Continue and steer

Keep the program ID. Ask the manager to continue, explain findings, change
approach, pause dispatch or stop. Reopening a program restores its records,
not a model conversation or physics world.

## Save research in Git

```bash
./hp-sim5-research-program save PROGRAM_ID --commit
```

This commits the program's result archive only; code changes are separate.
See [saving details](program-saving.md) for extra files and evidence limits.

Use [manager tools](program-manager.md) for lower-level commands,
[workers](program-workers.md) for dispatch, or `--help` for CLI options.
