# Live manager release gate

Status: blocked. Base source b15697fb50edd4ac2c358051c77ddce57abeacea.

Question: Can fixed saved HP3 measurements distinguish a useful calibration-fit
improvement from selection-only consensus changes?

The fit and CM-E1 routes remain not tested. The actual launcher failed at native
build before tools/client/science. Dedicated client and separate review failed
at read-only app-server startup; stock MCP initialize timed out. Zero fits,
physics steps, movement or collection ran. The pre-recorded plan allowed one
targeted fit, 25 evaluations and 30 points. Its zero physics/collection counts
were specific to saved-data fitting, not a global research restriction.

The live attempt found a real disabled-MCP transport defect, so the original
no-source-change plan was revised. The compact shared helper replaces complete
legacy server tables. Both manager/reviewer argv and generated attachment TOML
use valid disabled transports, including with inherited HTTP settings. Installed
client parsing and focused checks passed. Existing auth was reused; no API keys,
HOME/CODEX_HOME overrides, network install/approval waits or SDK patches were used.

Actual graph/lifecycle/fresh-host checks and their limitations are in report.md.
The final stopped program has zero scientific admissions. Partial startup evidence
and an unsuccessful separate review remain discoverable. Counterfactual withdrawal
did not retract a real measurement or turn imported material into reviewed facts.

After a usage-limit/chat pause, the parent resumed the authorized handoff directly,
rechecked current prerequisites and documented CLI commands, and retained passing
producer checks without a broad rerun. The finite driver's pause expiry is logged;
no runtime grant was extended. Next decision: complete task016 only where native
build, writable coding-client state and stock MCP work. Preserve the release block.
