# Subtask 1 (SUPPORT_HTTP): Host HTTP server mode without cpp-httplib

## Goal
Run `rrf_simulator` in a long-lived mode that serves `/machine/code` (and minimal `/machine/model`) by reusing RepRapFirmware’s built-in HTTP responder instead of pulling in `cpp-httplib`. Keep the host networking layer lightweight: a small POSIX/WinSock shim that feeds the existing `HttpResponder`/`NetworkResponder` stack, not the full Duet Software Framework.

## Constraints & Scope
- No new third-party HTTP libraries; prefer existing RRF networking code paths.
- Don’t pull in DSF/DuetWebServer; keep everything in the simulator process.
- Keep the shim small and host-only (`#if RRF_HOST_BUILD`), without touching board builds.
- Networking is disabled in `Features_Host.h` today; we must selectively re-enable just enough for HTTP and the HTTP G-code channel.

## Implementation Steps

1) Enable HTTP in the host feature set
- Flip `SUPPORT_HTTP` to `1` and introduce a host-only `HAS_NETWORKING` toggle (or define it in CMake) so `Networking/Network.cpp` and `HttpResponder.cpp` are compiled for host builds. Keep WiFi/LwIP/W5500 disabled.
- Expose the HTTP G-code channel by ensuring `SUPPORT_HTTP || HAS_SBC_INTERFACE` stays true in `GCodes.cpp` for host builds.

2) Add the minimal firmware networking sources to the host target
- Include in the generated sources list: `Networking/Network.cpp`, `Networking/NetworkResponder.cpp`, `Networking/NetworkBuffer.cpp`, `Networking/NetworkClient.cpp`, `Networking/HttpResponder.cpp`, and their headers. Exclude FTP/Telnet/MQTT.
- Ensure `RRF_HEADERS_TO_USE` contains the referenced networking headers (`Network.h`, `NetworkDefs.h`, `Socket.h`, `HttpResponder.h`, `NetworkResponder.h`, `NetworkBuffer.h`, `NetworkClient.h`).

3) Implement host shims for sockets and interfaces
- Add host-side implementations in `RRF/host/networking` (or `host/include/Networking`):
  - `SocketHost` implementing the interface expected by `Socket`/`NetworkResponder` using BSD sockets (listen/accept/read/write/close) with `#if RRF_HOST_BUILD` guards.
  - A minimal `NetworkInterfaceHost` that represents a single “Ethernet” interface, owns the listen socket, and exposes IP/port getters used by `Network` and `HttpResponder`.
  - A trimmed `NetworkResponderHost` loop that accepts connections and hands bytes to `HttpResponder::CharFromClient` and writes replies back.
- Keep buffering small (e.g., 16–64 KB per connection) and single-threaded to start; the existing responder’s state machine is already non-blocking.

4) Wire the responder into the host main loop
- In `RRF/host/src/main.cpp`, when `--server` (Subtask 2) is selected:
  - Initialize `Network`/`HttpResponder` once after `reprap.Init()`.
  - Start a small accept/poll loop (can be a thread) that drives `NetworkResponderHost::Spin()`; keep the existing `reprap.Spin()` loop untouched.
  - Keep signal handling/graceful shutdown as in the previous plan.

5) Map `/machine/code` to the existing `rr_gcode` path
- `HttpResponder` already parses `rr_gcode`. Adjust the host shim to route `POST /machine/code` to the same handler:
  - Option A: add an alias in `HttpResponder::ProcessRequest()` for `machine/code` that enqueues the body as a single G-code line on the HTTP channel and waits for a reply.
  - Option B: convert `machine/code` into an internal `rr_gcode` request with the body as `gcode=<body>`.
- Ensure content-type text/plain is accepted; ignore authentication/session keys in host mode.

6) Minimal `/machine/model` support
- Reuse `rr_model` handler; expose it under `/machine/model` by aliasing in `HttpResponder` (light change, host-only).
- If buffer pressure is an issue, cap flags to defaults and document that only a compact model is returned in host mode.

7) Keep capture plumbing unchanged
- Continue to use `HostCanCapture` for CAN logs; `/machine/code` responses can append motion data per later subtasks. No filesystem writes are needed for HTTP itself.

## Testing & Validation
- Unit-ish: add a host test that instantiates the host socket/responder classes and runs a short accept/send/recv cycle with a raw HTTP `POST /machine/code` sending `M114`, asserting a `200` reply containing axis positions.
- Integration script (bash):
  - Build simulator.
  - Launch `rrf_simulator --vsd RRF/run/vsd -c sys/config_slideprinter.g --server -p 8080 &`.
  - `curl -s http://localhost:8080/machine/code -d "M569.4 P40.0 T0.001"` → expect body contains `0.001000 Nm,`.
  - `curl -s http://localhost:8080/machine/model` → expect JSON with at least `move` present.
  - Kill server and ensure no crashes on shutdown.
- Determinism: run twice; responses and capture counts should match.

## Notes & Trade-offs
- Reusing `HttpResponder` keeps protocol compatibility and avoids a second HTTP parser, at the cost of small host socket shims.
- Keep the host networking shim self-contained; do not pull in LwIP/W5500/WiFi stacks.
- Authentication/session keys can be stubbed as no-ops for host mode to keep the code minimal.
