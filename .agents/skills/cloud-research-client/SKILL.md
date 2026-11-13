---
name: cloud-research-client
description: Run hp-sim5 research in Codex Cloud, including its optional simulation toolchain and environment setup.
---

Use the research-program skill at .agents/skills/research-program/SKILL.md.
The current cloud chat manages authenticated subagents directly. There is no
model queue, client bridge, extra login, or sequential-only cloud controller.
For runtime installation and sandbox details read research-program/platforms.md.

Existing environment setup commands scripts/install.sh and scripts/env.sh forward
to research-program/cloud/; no model-dispatch adapter remains.

Before starting a simulation runtime in a new shell, run
`source research-program/cloud/env.sh`. The supervisor inherits that shell's
tool paths; sourcing it later in a caller does not update an already running
supervisor. If a native build is stale, run `npm run build:native` in the sourced
shell or let world configuration rebuild it. Inspect the run's supervisor and
runtime logs if configuration fails. Stop the old runtime before restarting it
with a corrected environment. Saved-evidence research needs no physics setup.
