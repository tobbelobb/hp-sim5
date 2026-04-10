# Design Discussion — Klipper Integration for Autocal

## Current State
The autocal workflow is tightly coupled to RepRapFirmware (RRF). 
- **Python**: `autocal.py` and `_autocal_common.py` have hard-coded RRF constants (port 8081, `rrf_simulator` binary) and use `_send_rrf_gcode` for all communication (`autocal/_autocal_common.py:30-36`, `593-600`).
- **Node**: `collect_sweep_data.mjs` imports RRF-specific bridge components directly and assumes an HTTP-based `/machine/code` transport (`autocal/control/cli/collect_sweep_data.mjs:4-15`).
- **Simulator Lifecycle**: Python currently manages the `rrf_simulator` process during `--sim` runs, injecting `--no-spawn-rrf-simulator` into the Node collector args (`autocal/autocal.py:223-246`, `autocal/_autocal_common.py:448-458`).
- **Klipper**: A separate integration stack exists under `integrations/klipper/` but is not yet wired into the `autocal` command-line surface.

## Desired End State
The `autocal` command supports `--firmware klipper` (defaulting to `rrf`).
- **Unified CLI**: Users can run `python autocal/autocal.py --sim --firmware klipper` to execute the full auto-calibration loop against a simulated Klipper instance.
- **Robust Process Management**: Python owns the lifecycle of `klippy.py` and `klipper.elf`, ensuring they persist throughout the entire multi-sweep optimization loop and are cleaned up reliably on exit.
- **Abstracted Communication**: Both Python and Node use a firmware-agnostic interface to send G-code and retrieve machine state, allowing seamless switching between RRF and Klipper.
- **Verified Simulation**: The simulated hangprinter (`hangprinter_4`) correctly boots using the provided Klipper configs and responds to motion/encoder requests.

## Patterns to Follow
- **Strategy Pattern (Python)**: Follow the pattern of `optimizer_mode` normalization (`autocal/autocal.py:1330-1341`) to resolve a `FirmwareProvider` instance that encapsulates transport and simulator startup logic.
- **Bridge Factory (Node)**: Match the `createGcodeBridge` pattern but move to a factory in `collect_sweep_data.mjs` that selects between `RrfHttpBridge` and `KlipperTerminalBridge` based on the `--firmware` argument.
- **CLI Forwarding**: Continue the pattern of normalizing and appending arguments to `collector_args` before calling the Node subprocess (`autocal/autocal.py:1342-1355`).
- **Avoid**: Hard-coding firmware-specific strings like `rrf_server` or `m669` in high-level loop logic.

## Design Decisions

1.  **Firmware Strategy Pattern**:
    - Introduce a `FirmwareProvider` abstraction in `_autocal_common.py`.
    - Implement `RRFFirmwareProvider` and `KlipperFirmwareProvider` to handle G-code sending, readiness checks, and simulator process management.

2.  **Klipper Lifecycle Ownership (Python-led)**:
    - Python will manage the `klippy.py` and `klipper.elf` processes.
    - It will utilize the existing Node-based tools in `integrations/klipper/klippy_api_cli_config.mjs` by invoking them as subprocesses if necessary, but will pass a `--keep-alive` flag to prevent them from exiting prematurely.
    - Python's cleanup routine (`atexit` or similar) will be responsible for terminating these processes.
    - Any interactive prompts from Klipper setup (e.g., `sudo scripts/make-fake-pin-chip.sh`) will be piped through to the user's terminal.

3.  **Unified Node Entrypoint**:
    - Refactor `collect_sweep_data.mjs` to use a factory for bridge creation.
    - The bridge interface must support `sendGcodeLine` and return parsed motion/encoder data, hiding the difference between RRF's HTTP polling and Klipper's Unix-socket/WebSocket event stream.

4.  **Klipper Configuration**:
    - Default `hangprinter_4` Klipper config: `examples/klipper/hp3/printer-hp3-linux-mcu-with-buildup.cfg`.
    - Baseline fallback: `examples/klipper/hp3/printer-hp3-linux-mcu.cfg`.

5.  **Calibration Output**:
    - For Klipper, the final calibration values will be printed to the summary but **not** sent back to the firmware automatically. This avoids complex `printer.cfg` writes or assuming specific macro names.

## What We're NOT Doing
- **Automated Klipper Config Writing**: We will not attempt to modify `printer.cfg` programmatically after calibration.
- **Support for Other Firmwares**: This design specifically targets RRF and Klipper; other firmwares (Marlin, etc.) remain out of scope.
- **Refactoring RRF Core**: We are abstracting the *interface* to RRF, not rewriting the underlying RRF transport logic.

## Open Risks
- **Process Cleanup**: Ensuring `klipper.elf` (the host MCU) is killed reliably across all exit paths in Python (including SIGINT/SIGTERM) can be brittle.
- **Node-Python Coordination**: Passing `--keep-alive` to a Node helper while Python owns the PID requires careful synchronization to avoid zombie processes or port collisions.
- **Interactive Sudo**: If `make-fake-pin-chip.sh` requires `sudo` password entry, the subprocess pipe must be fully interactive (PTY) to avoid hanging.
