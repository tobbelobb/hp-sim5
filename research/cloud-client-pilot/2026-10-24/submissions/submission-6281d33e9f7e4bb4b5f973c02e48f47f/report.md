# Keyless cloud worker ran the launcher-attached native MCP

An authenticated cloud agent completed one bounded HP3 experiment through the existing launcher's attachment MCP without another login or API key. The real rust-native run executed 100 fixed 2 ms steps. Telemetry and the saved Rerun recording each contain 101 samples covering steps 0 through 100, and the recording blueprint is readable. This supports this cloud-client transport's production-tool feasibility. It does not establish calibration accuracy, automatic background dispatch, separate review or completion of task 016.

Topics: cloud agents, HP3, native physics, MCP, Rerun, existing login.

## Method and result

The parent manager dispatched assignment `cloud-native-operation`, attempt `attempt-d35a0ef8b1b54fc4a14bc9a1ba28aa9f`, through the existing research launcher. This actual cloud coding-agent session read the assignment and launcher instructions, then invoked `scripts/research_attach.py` with the private descriptor path. The descriptor token was not printed or copied into public artifacts. Stock JSON-RPC initialize and `tools/call run_experiment` reached the production MCP. No fake client or direct private physics implementation was used.

Run `8f977de942c64d7391f63c3198ba0c9d` completed at source revision `8acde0c21d5b9db8893115e3e79d82d9e41f98ca` using `rust-native`. It used the authored HP3 scene, no command changes, 100 steps at dt `0.002` s, and recording enabled. Both telemetry and RRD cable scalar coverage are exactly steps 0 through 100. The RRD contains 1,494,557 bytes and a readable blueprint.

Hypothesis: an already-authenticated cloud worker can operate the existing launcher's production tool without a new credential. Outcome: supported for this bounded tool call. Scientific calibration improvement and hardware accuracy were not tested. Reviewer acceptance and automatic platform-agent dispatch were not tested by this worker.

## Budgets and identities

Actual fresh steps: 100, aggregate simulated time: 0.2 s. Fresh fits: zero. Collection jobs: zero. Additional login/API keys: none. No tracked files or physics code changed. Source scene SHA-256: `3b0171abad47934f926bcf1cf72b6bfcc89b4a9c110bb749556d381f489b6e47`. Frozen command input identity: `4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945`. Python source identity: `d2ed374f1634fae5f721c4e4e8162e49d566d6b3ac015212f40df18d3a3fbf22`. Native artifact details remain in [production-result.json](production-result.json). Exact artifact SHA-256 values are in [validation.json](validation.json).

## Evidence and limits

- [Assignment](../../../cloud-research/client-queue/0020608d3dea4aceb73595b1db0244d1/prompt.md) records allocation and constraints.
- [Actual MCP requests and replies](mcp-transcript.json) show stock protocol initialization and the production tool response.
- [Reproducible probe](managed_mcp_check.py) attaches through the existing launcher and validates recording coverage.
- [Production result](production-result.json) links immutable run inputs, telemetry, snapshot and RRD.
- [Validation and hashes](validation.json) records source and artifact identities.

This was one bounded simulated idle experiment. It does not compare fitting algorithms, prove useful calibration, cover hardware, exercise interruption/recovery or finish the manager release. The parent chat still relays platform agent dispatch. The dedicated nested Codex CLI route was not used.
