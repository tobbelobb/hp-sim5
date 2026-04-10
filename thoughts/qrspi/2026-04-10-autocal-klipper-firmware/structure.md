# Structure Outline

## Approach
Abstract the firmware-specific logic into a `FirmwareProvider` (Python) and a `BridgeFactory` (Node). Python will manage the lifecycle of the Klipper simulator processes, ensuring they are ready before invoking the Node-based data collector, which will communicate via a unified bridge interface.

## Phase 1: CLI & Bridge Factory Foundation
Enable the `--firmware` argument across the stack and introduce the factory pattern in Node to select the appropriate communication bridge.

**Files**:
- `autocal/_autocal_common.py` (Parser update)
- `autocal/control/cli/collect_sweep_data.mjs` (Parser update, factory invocation)
- `autocal/control/primitives/bridge_factory.mjs` (New: exports `createBridge(firmware, options)`)

**Key changes**:
- `_add_collector_args(parser)`: adds `--firmware {rrf,klipper}`
- `createBridge(firmware: string, options: Object): GcodeBridge`: factory to return `RrfHttpBridge` or `KlipperTerminalBridge`.

**Verify**: `python autocal/autocal.py --help` shows `--firmware`; `node autocal/control/cli/collect_sweep_data.mjs --help` shows `--firmware`.

---

## Phase 2: Python Firmware Strategy & RRF Refactor
Introduce the `FirmwareProvider` abstraction in Python and migrate existing RRF logic into `RRFFirmwareProvider`.

**Files**:
- `autocal/_autocal_common.py` (Abstraction & RRF implementation)
- `autocal/autocal.py` (Refactor to use provider)

**Key changes**:
- `class FirmwareProvider`: Abstract base with `start_simulator()`, `wait_for_ready()`, `send_gcode()`.
- `class RRFFirmwareProvider(FirmwareProvider)`: Implements RRF-specific logic.
- `get_firmware_provider(firmware_name: str): FirmwareProvider`: Factory for Python.

**Verify**: `pytest autocal/tests/test_autocal_cli.py` and `autocal/tests/test_autocal_no_collect.py` pass (ensures no RRF regressions).

---

## Phase 3: Klipper Simulator Lifecycle (Python-led)
Implement Klipper-specific process management in Python, allowing it to boot and manage `klippy.py` and `klipper.elf`.

**Files**:
- `autocal/_autocal_common.py` (`KlipperFirmwareProvider`)
- `integrations/klipper/klippy_api_cli_config.mjs` (Support for `--keep-alive` if needed)

**Key changes**:
- `KlipperFirmwareProvider.start_simulator()`: Spawns `klippy.py` and `klipper.elf` using existing config paths.
- `KlipperFirmwareProvider.wait_for_ready()`: Probes Klipper's Unix socket for readiness.

**Verify**: `python autocal/autocal.py --sim --firmware klipper --no-collect` successfully boots Klipper and stays alive until the process is interrupted.

---

## Phase 4: Node Klipper Bridge Integration
Wire the existing `KlipperTerminalBridge` into the Node collector's factory and ensure it adheres to the expected data collection interface.

**Files**:
- `autocal/control/primitives/bridge_factory.mjs` (Hook up `KlipperTerminalBridge`)
- `autocal/control/cli/collect_sweep_data.mjs` (Refine bridge usage)

**Key changes**:
- `createBridge` now returns `createKlipperTerminalBridge(...)` when `firmware === 'klipper'`.
- Ensure `KlipperTerminalBridge` returns motion/encoder data in a format compatible with `collectSweepData()`.

**Verify**: Manually run `node autocal/control/cli/collect_sweep_data.mjs --firmware klipper --sim ...` against a running Klipper instance; check if `dataset.json` is generated correctly.

---

## Phase 5: Full Loop E2E Integration
Final wiring to allow the full auto-calibration loop to run against Klipper.

**Files**:
- `autocal/autocal.py` (Final glue logic)
- `autocal/_autocal_common.py` (Final helper refinements)

**Key changes**:
- `full_auto_loop()` uses `FirmwareProvider` for all lifecycle and communication.
- `_emit_summary_and_send()` skips auto-send for Klipper as per design.

**Verify**: `python autocal/autocal.py --sim --firmware klipper` completes a full calibration cycle with at least one sweep iteration.

---

## Testing Checkpoints
1. **Post-Phase 1**: Both CLIs accept `--firmware`, but Klipper throws "not implemented".
2. **Post-Phase 2**: RRF full-auto loop works exactly as before.
3. **Post-Phase 3**: Klipper simulator boots and cleans up reliably from Python.
4. **Post-Phase 4**: Node collector can fetch real data from a Klipper instance.
5. **Post-Phase 5**: Full `autocal.py` loop completes using Klipper.
