  Architecture Overview

  ┌─────────────────────────────────────────────────────────────────┐
  │                     HTTP Client (curl/JS)                       │
  │                            │                                    │
  │                    POST /machine/code                           │
  │                     "M569.4 P40.0 T0.001"                       │
  └────────────────────────────┬────────────────────────────────────┘
                               │
  ┌────────────────────────────▼────────────────────────────────────┐
  │              rrf_simulator (Server Mode)                        │
  │  ┌─────────────────┐   ┌─────────────────┐   ┌───────────────┐  │
  │  │   HTTP Server   │──▶│  GCodeInjector  │──▶│   GCodes      │  │
  │  │   (Subtask 1)   │   │   (Subtask 2)   │   │   System      │  │
  │  └─────────────────┘   └────────┬────────┘   └───────┬───────┘  │
  │                                 │                    │          │
  │                                 │              ┌─────▼─────┐    │
  │                                 │              │ M569.4    │    │
  │                                 │              │ HOST_BUILD│    │
  │                                 │              │ Handler   │    │
  │                                 │              │(Subtask 3)│    │
  │                                 │              └─────┬─────┘    │
  │                                 │                    │          │
  │                         ┌───────▼────────────────────▼───────┐  │
  │                         │         CanCapture                 │  │
  │                         │    (Memory Buffer - Subtask 4)     │  │
  │                         └───────────────────┬────────────────┘  │
  └─────────────────────────────────────────────┬───────────────────┘
                                                │
                       Response: "0.001000 Nm,\n---MOTION---\nT,40,0.001"
                                                │
  ┌─────────────────────────────────────────────▼───────────────────┐
  │                  JavaScript Physics Simulation                  │
  │  ┌─────────────────┐   ┌──────────────────┐   ┌──────────────┐  │
  │  │  RrfHttpBridge  │──▶│RemoteSpoolSystem │──▶│StepperMotor  │  │
  │  │   (Subtask 5)   │   │ (existing + new  │   │  System      │  │
  │  └─────────────────┘   │  commands)       │   │ (Subtask 6)  │  │
  │                        └──────────────────┘   └──────────────┘  │
  └─────────────────────────────────────────────────────────────────┘

  The 7 Subtasks

  | #   | File                                      | Description                                    |
  |-----|-------------------------------------------|------------------------------------------------|
  | 1   | http_endpoint_implementation_subtask_1.md | Enable SUPPORT_HTTP and host http server       |
  | 2   | http_endpoint_implementation_subtask_2.md | G-code injection and response capture          |
  | 3   | http_endpoint_implementation_subtask_3.md | Simplified M569.4 handler (#if RRF_HOST_BUILD) |
  | 4   | http_endpoint_implementation_subtask_4.md | Motion command streaming (in-memory buffer)    |
  | 5   | http_endpoint_implementation_subtask_5.md | JavaScript RrfHttpBridge class                 |
  | 6   | http_endpoint_implementation_subtask_6.md | Torque mode in StepperMotorSystem              |
  | 7   | http_endpoint_implementation_subtask_7.md | Integration tests and validation               |

  Key Design Decisions

  1. Simplified M569.4: Instead of using the complex #if DUAL_CAN code that talks to actual ODrive boards, we add a short #if RRF_HOST_BUILD block that:
    - Tracks torque state per driver (40-43)
    - Returns formatted responses ("0.001000 Nm," or "pos_mode,")
    - Emits events for the physics simulation
  2. Response Format: Motion data is appended to the G-code reply with a ---MOTION--- delimiter, making it easy to parse while maintaining compatibility with the existing response format.
  3. Physics Torque Mode: The StepperMotorComponent gains two new fields (torqueMode, targetTorque). When enabled, constant torque is applied instead of position-seeking behavior.
  4. Threading Model: The HTTP server runs on the main thread while reprap.Spin() runs on a background thread, with proper synchronization through the GCodeInjector.

  Each subtask includes:
  - Detailed implementation code
  - File paths and line references
  - Unit test examples
  - Integration test scripts
  - Validation criteria
