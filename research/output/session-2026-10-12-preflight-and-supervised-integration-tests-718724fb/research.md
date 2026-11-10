# Preflight and supervised integration tests

Objective: implement research/prompts/tooling_improvement_3.txt.

Hypothesis: bounded supervisor checks and fixed integration jobs work through MCP while agent command sockets are denied.

Constraints and budgets: no movement/reset/doctor/full-auto smoke in the attached HP4 world. Preflight 15 s plus cleanup; integration jobs at most 900 s, validation uses 600 s. Test fixtures can move their isolated worlds. Four validation attempts preserve all directories and recordings.

Accepted steering: implement the supplied feature prompt, using .venv/bin/python and existing supervisor/configuration interfaces.

Evidence so far:
- [Actual command probe](command-network.json): AF_INET socket creation denied by the command sandbox, EPERM; binding/HTTP remain untested.
- [Initial preflight](initial-mcp-preflight.json): passed in 0.837 s with readable finalized tool-check RRD, SDK client denied socket/connect syscalls by Linux seccomp.
- [Initial research suite](initial-research-services-result.json): 4 failed / 123 passed / 14 deselected. Long inherited TMPDIR broke RRF's 120-character VSD path constraint; nested catalog assertions assumed external paths.
- [Second research suite](second-research-services-result.json): 2 failed / 125 passed / 15 deselected. The catalog intentionally excludes nested test session artifacts; updated test expectation.
- [Third research suite](third-research-services-result.json): 2 failed / 128 passed / 14 deselected. Detached-service cleanup tests exposed inherited parent registries; each nested job now owns its registry, and parent cleanup traverses nested registries.
- [Latest preflight](mcp-preflight.json): passed in 0.737 s. Headless suite passed, 26 tests / 2 deselected in 20.17 s. The final research-services suite passed 130 tests (14 deselected) in 37.42 s; explicit cancellation and timeout passed.

Changes: non-moving MCP/CLI preflight, explicit command socket probe, fixed named start/status/cancel test jobs, credential-free environment, isolated logs/JUnit/pytest artifacts, detached process group ownership and cleanup. Normal startup does not call doctor/preflight. Network-enabled default and explicit false override are preserved.

Next decision: none required. Implementation, validation, report and catalog are complete. Future work could verify other platforms and a live Codex attachment under managed policy.

No calibration-accuracy hypothesis is tested by this tooling work.

Final decision: objective complete. Preflight 0.737 s; 26 headless and 130 research-service tests passed, real MCP cancellation and timeout passed, disconnected supervisor blocked as expected. CLI preflight 0.736 s. Attached HP4 world/recording unchanged. No calibration accuracy claim. Detailed evidence and limitations in [report](report.md).
