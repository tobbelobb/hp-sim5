# Implementation Plan

## Overview
The `autocal` command supports `--firmware klipper` (defaulting to `rrf`), with Python owning the lifecycle of the Klipper simulator and Node providing an abstracted communication bridge for sweep data collection.

## Phase 1: CLI & Bridge Factory Foundation

### Changes

#### 1. Python CLI Parser Update
**File**: `autocal/_autocal_common.py`
**Action**: Modify `_add_collector_args` to include the `--firmware` argument.

```python
def _add_collector_args(parser: argparse.ArgumentParser) -> None:
    # ... existing args ...
    parser.add_argument(
        "--firmware",
        choices=["rrf", "klipper"],
        default="rrf",
        help="Firmware flavor to target (default: rrf)",
    )
```

#### 2. Node Collector CLI Parser Update
**File**: `autocal/control/cli/collect_sweep_data.mjs`
**Action**: Update `parseBridgeArgs` to include the `--firmware` argument.

```javascript
export function parseBridgeArgs(argv) {
  const args = {
    firmware: 'rrf',
    // ... rest of args ...
  };
  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--firmware') {
      args.firmware = argv[++i] || 'rrf';
    }
    // ...
  }
}
```

#### 3. New Bridge Factory
**File**: `autocal/control/primitives/bridge_factory.mjs`
**Action**: Create a factory to return the appropriate bridge instance.

```javascript
import { createGcodeBridge } from '../../../integrations/rrf/rrfSimulatorBridge.mjs';
import { KlippyApiClient } from '../../../integrations/klipper/klippyApiClient.js';
import { KlippyRuntimeState } from '../../../integrations/klipper/klippyRuntimeState.js';
import { createKlipperTerminalBridge } from '../../../integrations/klipper/klipperTerminalBridge.js';

export async function createBridge(firmware, options = {}) {
  if (firmware === 'klipper') {
    const client = new KlippyApiClient({ socketPath: options.socketPath });
    const klippyState = new KlippyRuntimeState({ client });
    // Note: KlippyApiClient.start() and connection wait should happen here or be handled by the bridge
    return createKlipperTerminalBridge({
      client,
      klippyState,
      wsPort: options.wsPort,
      quiet: options.quiet,
      configPath: options.configPath,
    });
  }
  return createGcodeBridge(options);
}
```

### Verification
#### Automated
- [x] `python autocal/autocal.py --help` shows `--firmware`
- [x] `node autocal/control/cli/collect_sweep_data.mjs --help` shows `--firmware`

#### Manual
- [x] Run `node autocal/control/cli/collect_sweep_data.mjs --firmware klipper` and verify it fails with a "requires KlippyApiClient" or similar if not implemented correctly, but accepts the flag.

---

## Phase 2: Python Firmware Strategy & RRF Refactor

### Changes

#### 1. FirmwareProvider Abstraction
**File**: `autocal/_autocal_common.py`
**Action**: Introduce `FirmwareProvider` and `RRFFirmwareProvider`.

```python
class FirmwareProvider(ABC):
    @abstractmethod
    def start_simulator(self, port: int, sim_config: Optional[str] = None) -> Optional[subprocess.Popen]: pass
    @abstractmethod
    def wait_for_ready(self, target: str, timeout_s: float = 7.0) -> None: pass
    @abstractmethod
    def send_gcode(self, target: str, gcode: str, timeout_s: float = 5.0) -> str: pass

class RRFFirmwareProvider(FirmwareProvider):
    # Implements using _start_rrf_simulator, _wait_for_rrf_server, _send_rrf_gcode
```

#### 2. Refactor autocal.py to use Provider
**File**: `autocal/autocal.py`
**Action**: Update `full_auto_loop` and `_emit_summary_and_send`.

```python
provider = get_firmware_provider(args.firmware)
sim_process = provider.start_simulator(target_port, sim_config=sim_config)
provider.wait_for_ready(target_server)
```

### Verification
#### Automated
- [x] `pytest autocal/tests/test_autocal_cli.py` passes
- [x] `pytest autocal/tests/test_autocal_no_collect.py` passes

#### Manual
- [x] Run `python autocal/autocal.py --sim --no-collect` and verify it still works for RRF.

---

## Phase 3: Klipper Simulator Lifecycle (Python-led)

### Changes

#### 1. KlipperFirmwareProvider Implementation
**File**: `autocal/_autocal_common.py`
**Action**: Implement Klipper-specific lifecycle management.

```python
class KlipperFirmwareProvider(FirmwareProvider):
    def start_simulator(self, port: int, sim_config: Optional[str] = None):
        # Spawn scripts/run_klippy_api_mode.sh or similar
        return subprocess.Popen(["./scripts/run_klippy_api_mode.sh"], ...)
    
    def wait_for_ready(self, target: str, timeout_s: float = 7.0):
        # Probe Unix socket /tmp/klippy_uds
        pass

    def send_gcode(self, target: str, gcode: str, timeout_s: float = 5.0):
        # Klipper send usually happens via Node bridge, but Python might need it for final check
        # Use a minimal JSON-RPC over UDS if needed, or skip if only Node needs to send.
        pass
```

### Verification
#### Automated
- [ ] `python autocal/autocal.py --sim --firmware klipper --no-collect` successfully boots Klipper.

#### Manual
- [ ] Check `ps aux | grep klippy` to see if it's running.
- [ ] Interrupt Python and verify Klipper processes are killed.

---

## Phase 4: Node Klipper Bridge Integration

### Changes

#### 1. Hook up KlipperTerminalBridge in Factory
**File**: `autocal/control/primitives/bridge_factory.mjs`
**Action**: Ensure `createBridge` returns a fully functional Klipper bridge.

#### 2. Refactor collect_sweep_data.mjs to use BridgeFactory
**File**: `autocal/control/cli/collect_sweep_data.mjs`
**Action**: Replace `createGcodeBridge` with `createBridge`.

```javascript
const bridgeCtx = await createBridge(args.firmware, {
  server: targetServer,
  socketPath: args.socket || DEFAULT_KLIPPY_SOCKET_PATH,
  wsPort: args.noWs ? 0 : args.wsPort,
  quiet: args.quiet,
  configPath: args.config || DEFAULT_KLIPPY_CONFIG_PATH,
  encoderTimeoutMs,
});
```

### Verification
#### Automated
- [ ] `node autocal/control/cli/collect_sweep_data.mjs --firmware rrf --sim ...` still works.

#### Manual
- [ ] Manually run `node autocal/control/cli/collect_sweep_data.mjs --firmware klipper --sim --machine-type hangprinter_4` and verify it can collect at least one point.

---

## Phase 5: Full Loop E2E Integration

### Changes

#### 1. Final Glue Logic
**File**: `autocal/autocal.py`
**Action**: Ensure `full_auto_loop` passes correctly normalized args to the Node collector.

#### 2. Skip M669 for Klipper
**File**: `autocal/autocal.py`
**Action**: Update `_emit_summary_and_send` to check firmware.

```python
if firmware == 'rrf':
    _send_rrf_gcode(rrf_server, m669)
else:
    _log_line("; --firmware klipper set; skipping M669 send.")
```

### Verification
#### Automated
- [ ] `python autocal/autocal.py --sim --firmware klipper` completes a full calibration cycle.

#### Manual
- [ ] Verify `dataset.json` contains valid data from Klipper.
- [ ] Verify `calibration.gcode` is generated and printed in the summary.
