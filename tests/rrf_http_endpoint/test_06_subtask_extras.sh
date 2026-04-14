#!/usr/bin/env bash
set -euo pipefail

echo "=== Test 6: HTTP Subtask Extras ==="

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BIN="$ROOT/RRF/build/rrf_simulator"
VSD="$ROOT/RRF/run/vsd"
CFG="$VSD/sys/config_slideprinter.g"
PORT="${PORT:-8080}"
LOG_FILE="${RRF_HTTP_LOG_FILE:-/tmp/rrf_http_$(basename "$0" .sh).log}"
ENDPOINT="http://localhost:${PORT}/machine/code"

fail() {
    echo "FAIL: $1"
    if [[ "${RRF_HTTP_DEBUG:-}" != "1" ]]; then
        echo "  (rrf_simulator log: $LOG_FILE)"
    fi
    exit 1
}

curl_post() {
    local data="$1"
    local response
    response=$(curl -s "$ENDPOINT" -d "$data" -H "Content-Type: text/plain") || fail "curl failed for '$data'"
    echo "$response"
}

if [[ "${RRF_HTTP_DEBUG:-}" == "1" ]]; then
    "$BIN" --vsd "$VSD" -c "$CFG" --server -p "$PORT" &
else
    "$BIN" --vsd "$VSD" -c "$CFG" --server -p "$PORT" >"$LOG_FILE" 2>&1 &
fi
SERVER_PID=$!
cleanup() {
    kill "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3

RESPONSE=$(curl_post "M999999")
[[ "$RESPONSE" == *"Command is not supported"* ]] || fail "Expected unsupported command response (got: $RESPONSE)"
echo "  Unsupported command: OK"

RESPONSE=$(curl_post $'M115\nM114')
[[ "$RESPONSE" == *"FIRMWARE_NAME"* ]] || fail "Expected multi-command response (got: $RESPONSE)"
echo "  Multi-command payload: OK"

curl_post "G92 X0 Y0 Z0" > /dev/null
resp_x=$(curl_post "G1 X100")
[[ "$resp_x" == *"---MOTION---"* ]] || fail "G1 X100 did not return motion data"
motion_x=$(echo "$resp_x" | sed -n '/---MOTION---/,$p' | tail -n +3 | tr -d '\r')
first_idx_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{print $1; exit}')
last_idx_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{val=$1} END{print val}')
count_x=$(echo "$motion_x" | awk -F',' '/^[0-9]+/{c++} END{print c+0}')
[[ -n "${first_idx_x:-}" && -n "${last_idx_x:-}" && -n "${count_x:-}" ]] || fail "Could not parse CAN indices for G1 X100"
(( last_idx_x - first_idx_x + 1 == count_x )) || fail "Missing CAN packets inside G1 X100 motion capture"

echo "  Motion capture continuity (X): OK"

resp_y=$(curl_post "G1 Y100")
[[ "$resp_y" == *"---MOTION---"* ]] || fail "G1 Y100 did not return motion data"
motion_y=$(echo "$resp_y" | sed -n '/---MOTION---/,$p' | tail -n +3 | tr -d '\r')
first_idx_y=$(echo "$motion_y" | awk -F',' '/^[0-9]+/{print $1; exit}')
[[ -n "${first_idx_y:-}" ]] || fail "Could not parse CAN indices for G1 Y100"
(( first_idx_y == last_idx_x + 1 )) || fail "Motion capture dropped CAN packets between sequential moves"

echo "  Motion capture continuity (X->Y): OK"

if command -v node &> /dev/null; then
    echo "  Running JS subtask checks"
    (cd "$ROOT" && PORT="$PORT" node --no-warnings --input-type=module <<'NODE'
import { RrfHttpBridge } from './integrations/rrf/rrfHttpBridge.js';
import { RemoteSpoolSystem } from './example_apps/js/slideprinter/slideprinter_common.js';

const assertOk = (cond, msg) => {
    if (!cond) {
        throw new Error(msg);
    }
};

const spool = new RemoteSpoolSystem();
const port = process.env.PORT || 8080;
const bridge = new RrfHttpBridge({
    baseUrl: `http://localhost:${port}`,
    remoteSpoolSystem: spool,
});

const queueBeforeMove = spool.getQueueLength();
const moveResp = await bridge.sendGCode('G1 X20');
assertOk(moveResp.motion.some((m) => m.type === 'Motion'), 'bridge move missing motion data');
const queueAfterMove = spool.getQueueLength();
assertOk(queueAfterMove > queueBeforeMove, 'motion commands were not enqueued');
const moveCmd = spool.commands.find((cmd) => cmd.type === 'Move');
const moveHasAxis = Boolean(moveCmd && (
    (moveCmd.axes && Object.values(moveCmd.axes).some((v) => typeof v === 'number'))
    || Object.keys(moveCmd).some((key) => key.length === 1 && key !== 'E' && typeof moveCmd[key] === 'number')
));
assertOk(moveHasAxis, 'Move command missing axis data');

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
        text: async () => `-0.000030 Nm,\n---MOTION---\n{"capture_version":1}\n0,40,0,20793,0,0,10\nT,40,-0.000030`,
    }),
});

await torqueBridge.sendGCode('M569.4 P40.0 T0.001');
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

console.log('  JS subtask checks: OK');
NODE
    )
else
    echo "  SKIP: Node.js not available for JS subtask checks"
fi

echo "PASS: HTTP subtask extras"
