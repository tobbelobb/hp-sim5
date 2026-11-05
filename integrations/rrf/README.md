# Simulator encoder reply budgets

`createGcodeBridge` gives M569.3 simulator encoder replies a 5000 ms budget by default. Set `encoderTimeoutMs` explicitly when a caller needs longer, for example `createGcodeBridge({ encoderTimeoutMs: 120000 })`. The command's `timeout` can shorten that budget. A command with `timeout: 80` still waits at most 80 ms for its encoder reply.

The direct `sendEncoderRequest` helper retains its separate 2000 ms default. Missing replies time out. Queries with no connected client fail immediately; in-flight disconnects remain bounded by the configured timeout. Replies route by request ID. The change does not alter the reported encoder values or physical settling rules.

The regression suite uses real WebSocket clients and a local HTTP firmware stub. It covers a valid response delayed six seconds, the five-second default, shorter command and encoder budgets, disconnects and concurrent request IDs. No hardware timing or full-autocal reliability claim follows from those checks.
