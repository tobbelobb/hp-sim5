#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$ROOT/RRF/build/rrf_simulator"
CFG="$ROOT/RRF/run/vsd/sys/config_slideprinter.g"
VSD="$ROOT/RRF/run/vsd"
PORT=8080

"$BIN" --vsd "$VSD" -c "$CFG" --server -p "$PORT" > /dev/null 2>&1 &
PID=$!
cleanup(){ kill $PID 2>/dev/null || true; }
trap cleanup EXIT

sleep 2

fail() {
  echo "FAIL: $1"
  exit 1
}

check() {
  local expected=$2 payload=$1
  resp=$(curl -sS -X POST "http://localhost:$PORT/machine/code" -H "Content-Type: text/plain" --data "$payload")
  echo "Sent: \"$payload\""
  echo "Got: $resp"
  if [[ "$resp" != *"$expected"* ]]; then
    fail "Expected \"$expected\" in response"
  fi
  echo "Success"
}

check "M115" "FIRMWARE_NAME"
check "M114" "X:"
check "M999999" "M999999: Command is not supported"
check $'M115\nM114' "FIRMWARE_NAME"
check "M569.4 P40.0 T0.001" "-0.000030 Nm"
check "M569.4 P40.0 T0" "pos_mode"
check "M569.4 P40.0:41.0 T0" "pos_mode, pos_mode"
check "M569.4 P40.0" "Error"

# Motion capture should include the full set of CAN frames across sequential moves.
curl -sS -X POST "http://localhost:$PORT/machine/code" -H "Content-Type: text/plain" --data "G92 X0 Y0 Z0" > /dev/null
resp_x=$(curl -sS -X POST "http://localhost:$PORT/machine/code" -H "Content-Type: text/plain" --data "G1 X100")
[[ "$resp_x" == *"---MOTION---"* ]] || fail "G1 X100 did not return motion data"
motion_x=$(echo "$resp_x" | sed -n '/---MOTION---/,$p' | tail -n +3 | tr -d '\r')
first_idx_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{print $1; exit}')
last_idx_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{val=$1} END{print val}')
count_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{c++} END{print c+0}')
[[ -n "${first_idx_x:-}" && -n "${last_idx_x:-}" && -n "${count_x:-}" ]] || fail "Could not parse CAN indices for G1 X100"
(( last_idx_x - first_idx_x + 1 == count_x )) || fail "Missing CAN packets inside G1 X100 motion capture"

resp_y=$(curl -sS -X POST "http://localhost:$PORT/machine/code" -H "Content-Type: text/plain" --data "G1 Y100")
[[ "$resp_y" == *"---MOTION---"* ]] || fail "G1 Y100 did not return motion data"
motion_y=$(echo "$resp_y" | sed -n '/---MOTION---/,$p' | tail -n +3 | tr -d '\r')
first_idx_y=$(echo "$motion_y" | awk -F',' '/^[0-9]+/{print $1; exit}')
[[ -n "${first_idx_y:-}" ]] || fail "Could not parse CAN indices for G1 Y100"
(( first_idx_y == last_idx_x + 1 )) || fail "Motion capture dropped CAN packets between sequential moves"

echo "Running RrfHttpBridge tests"
PORT=$PORT node --no-warnings --input-type=module <<'NODE'
import { RrfHttpBridge } from './examples/js/slideprinter/rrfHttpBridge.js';
import { RemoteSpoolSystem } from './examples/js/slideprinter/slideprinter_common.js';

const assertOk = (cond, msg) => {
    if (!cond) {
        throw new Error(msg);
    }
};

const torqueEvents = [];
const spool = new RemoteSpoolSystem();
const port = process.env.PORT || 8080;
const bridge = new RrfHttpBridge({
    baseUrl: `http://localhost:${port}`,
    remoteSpoolSystem: spool,
    onTorqueModeChange: (driver, axis, torqueNm) => {
        torqueEvents.push({ driver, axis, torqueNm });
    },
});

const fw = await bridge.getFirmwareInfo();
console.log('M115 reply:', fw.reply);
assertOk(fw.reply.includes('FIRMWARE_NAME'), 'bridge M115 missing firmware info');

const torqueResp = await bridge.sendGCode('M569.4 P40.0 T0.001');
console.log('Torque reply:', torqueResp.reply);
assertOk(torqueResp.reply.includes('-0.000030 Nm'), 'bridge torque reply missing value');

const queueBeforeMove = spool.getQueueLength();
const moveResp = await bridge.sendGCode('G1 X20');
console.log('Move motion items:', moveResp.motion.length);
assertOk(moveResp.motion.some((m) => m.type === 'Motion'), 'bridge move missing motion data');
const queueAfterMove = spool.getQueueLength();
assertOk(queueAfterMove > queueBeforeMove, 'motion commands were not enqueued');
const moveCmd = spool.commands.find((cmd) => cmd.type === 'Move');
const moveHasAxis = Boolean(moveCmd && (
    (moveCmd.axes && Object.values(moveCmd.axes).some((v) => typeof v === 'number'))
    || Object.keys(moveCmd).some((key) => key.length === 1 && key !== 'E' && typeof moveCmd[key] === 'number')
));
assertOk(moveHasAxis, 'Move command missing axis data');

// Ensure torque-mode callbacks fire on motion payloads regardless of live server output
const callbackEvents = [];
const fakeSpool = new RemoteSpoolSystem();
const torqueBridge = new RrfHttpBridge({
    baseUrl: 'http://localhost:0',
    remoteSpoolSystem: fakeSpool,
    onTorqueModeChange: (driver, axis, torqueNm) => callbackEvents.push({ driver, axis, torqueNm }),
    fetchImpl: async () => ({
        ok: true,
        status: 200,
        statusText: 'OK',
        text: async () => `-0.000030 Nm,
---MOTION---
{"capture_version":1}
0,40,0,20793,0,0,10
T,40,-0.000030`,
    }),
});

const torqueResult = await torqueBridge.sendGCode('M569.4 P40.0 T0.001');
assertOk(callbackEvents.length === 1 && Math.abs(callbackEvents[0].torqueNm + 0.00003) < 1e-6,
    'torque mode callback not triggered on torque event');
assertOk(fakeSpool.commands.some((cmd) => cmd.type === 'SetTorqueMode' && cmd.axis === 'A'),
    'torque mode command not queued from torque event');

const timeoutBridge = new RrfHttpBridge({
    baseUrl: 'http://localhost:0',
    fetchImpl: (url, { signal } = {}) => new Promise((resolve, reject) => {
        const timer = setTimeout(() => resolve({
            ok: true,
            status: 200,
            statusText: 'OK',
            text: async () => 'slow ok',
        }), 50);
        if (signal) {
            signal.addEventListener('abort', () => {
                clearTimeout(timer);
                const err = new Error('Aborted');
                err.name = 'AbortError';
                reject(err);
            });
        }
    }),
});

let timeoutThrown = false;
try {
    await timeoutBridge.sendGCode('G4 P1', { timeout: 5 });
} catch (err) {
    timeoutThrown = true;
    assertOk(/timed out/i.test(err.message), 'timeout error message unexpected');
}
assertOk(timeoutThrown, 'timeout error was not thrown');

const errorBridge = new RrfHttpBridge({
    baseUrl: 'http://localhost:0',
    fetchImpl: async () => ({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error',
        text: async () => 'oops',
    }),
});

let errorThrown = false;
try {
    await errorBridge.sendGCode('M999');
} catch (err) {
    errorThrown = true;
    assertOk(/HTTP 500/.test(err.message), 'HTTP error message missing status');
}
assertOk(errorThrown, 'error responses should throw');
NODE

echo "PASS: HTTP injector tests"
