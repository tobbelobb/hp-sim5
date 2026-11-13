---
name: research-program
description: Run or resume an hp-sim5 research program using the current coding platform's agents, a durable agenda, optional simulation tools, and fresh evidence review. Use for directed or continuing research investigations.
---

The current conversation manages the program. Use its authenticated subagents;
do not launch nested coding clients. Program records describe investigations, not
running actors. The platform owns actor IDs, steering, waiting, cancellation, and
conversation resumption. Read research-program/README.md for the small CLI contract.

```bash
./hp-sim5-research-program --question 'The human research question'
./hp-sim5-research-program --resume PROGRAM_ID
./hp-sim5-research-program --resume PROGRAM_ID --agent 'A useful independent question'
```

Opening returns manager instructions and a task. Commissioning returns researcher
instructions, a task, a stable ID, and an artifact path. Pass these once to a fresh
platform agent. Independent researchers may run in parallel. Give code-writing
researchers isolated workspaces using the platform, with dependencies prepared;
read research-program/platforms.md for T3, Codex Cloud, or Anthropic setup. Keep each
researcher's artifact path stable within its workspace; return its location to the
manager. Do not give workers the manager conversation or this manager procedure.

Researchers can work without physics. Each can start its own tools when needed:

```bash
./hp-sim5-research-agent --resume INVESTIGATION_ID --start-runtime
```

The returned tool_connection supports scripts/research_tools.py list/call without
command sockets; platform instructions explain its use. Each runtime has its own
world and evidence. Configuring or running one researcher's world does not affect
another's. Stop owned runtimes with --stop-runtime before the actor finishes;
platform cancellation alone is not confirmation that detached services stopped.
Inspect --runtime-status and retain partial evidence. After an uncertain mutation
timeout, inspect status before retrying.

Choose consequential, tractable questions. Let researchers own methods. Explore
alternatives, then concentrate on promising directions. Reconsider agenda.md after
persistent surprises, strong results, diminishing returns, or changed priorities;
do not merely commission more of the same experiment. A useful negative result can
complete an investigation. Read research-corpus when interpreting or publishing findings.

Use a fresh reviewer for consequential claims. The CLI can seal selected files and
the committed candidate for review. Give the returned handoff to an independent
agent; scientific acceptance requires its actual evidence-based assessment, not a
successful process exit or intact checksums. Inspect discrepancies and run relevant
checks on the final combined candidate. Physics changes require npm run test:physics.
Delivery uses the platform's ordinary Git/PR workflow and the human's authority;
this program does not automatically merge, push, or mark science successful.
