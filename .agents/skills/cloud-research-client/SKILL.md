---
name: cloud-research-client
description: Compatibility entry for launching or resuming an hp-sim5 research program in an authenticated cloud chat.
---

Use the research-program skill at .agents/skills/research-program/SKILL.md.
The current cloud chat manages authenticated subagents directly. There is no
model queue, client bridge, extra login, or sequential-only cloud controller.
For runtime installation and sandbox details read research-program/platforms.md.

Existing environment setup commands scripts/install.sh and scripts/env.sh forward
to research-program/cloud/; no model-dispatch adapter remains.
