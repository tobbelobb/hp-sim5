---
name: cloud-research-client
description: Operate hp-sim5 research programs in an authenticated cloud chat without another login or API key, using real cloud subagents for bounded worker and reviewer requests.
---

For an explicitly selected cloud research program, follow
`scripts/cloud/prompts/research-manager.txt` to open the program and load its
shared workflow and cloud supplement. To resume, use `--cloud --program ID`
in place of `--cloud --question TEXT` in that entry command.

Availability of this skill does not select cloud mode. The entry command does.
