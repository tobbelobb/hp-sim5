let RrfCommander;
let FileFormat;

const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);

function makeSingleEntryBinaryCan({
  time = 0,
  motorId = 40,
  steps = 1,
  accelTicks = 100,
  steadyTicks = 0,
  decelTicks = 0,
  acceleration = 0,
  deceleration = 0,
} = {}) {
  const MASK_ALL_CONTEXT_COLUMNS = 0x3f;
  const ENTRY_COUNT = 1;
  const buffer = new ArrayBuffer(2 + 8 + 4 + 4 + 4 + 4 + 4 + 1 + 1);
  const view = new DataView(buffer);
  let offset = 0;

  view.setUint8(offset, MASK_ALL_CONTEXT_COLUMNS);
  offset += 1;
  view.setUint8(offset, ENTRY_COUNT);
  offset += 1;

  const lower = time >>> 0;
  const upper = Math.floor(time / 0x1_0000_0000) >>> 0;
  view.setUint32(offset, lower, true);
  offset += 4;
  view.setUint32(offset, upper, true);
  offset += 4;

  view.setInt32(offset, accelTicks, true);
  offset += 4;
  view.setInt32(offset, steadyTicks, true);
  offset += 4;
  view.setInt32(offset, decelTicks, true);
  offset += 4;
  view.setFloat32(offset, acceleration, true);
  offset += 4;
  view.setFloat32(offset, deceleration, true);
  offset += 4;

  view.setUint8(offset, motorId & 0x3f);
  offset += 1;
  view.setInt8(offset, steps);

  return new Uint8Array(buffer);
}

function streamFromUint8Array(bytes) {
  let emitted = false;
  return new ReadableStream({
    pull(controller) {
      if (emitted) {
        controller.close();
        return;
      }
      controller.enqueue(bytes);
      emitted = true;
    },
  });
}

async function withImmediateTimeout(fn) {
  const originalSetTimeout = globalThis.setTimeout;
  globalThis.setTimeout = (callback) => {
    callback();
    return 0;
  };
  try {
    return await fn();
  } finally {
    globalThis.setTimeout = originalSetTimeout;
  }
}

async function collectBinaryMoves(binaryCan, { dt = null } = {}) {
  const commander = new RrfCommander();
  const moves = [];
  commander.sendCommand = async (command) => {
    if (command?.type === 'Move') {
      moves.push(command);
    }
  };
  commander.setAsapMode(true);
  if (Number.isFinite(dt) && dt > 0) {
    commander.setDt(dt);
  }
  await withImmediateTimeout(async () => {
    await commander.run(streamFromUint8Array(binaryCan), FileFormat.RRF_CAN_BINARY);
  });
  return moves;
}

describe('RrfCommander binary CAN compatibility', () => {
  beforeAll(async () => {
    if (!globalThis.self) {
      globalThis.self = {};
    }
    if (typeof globalThis.self.addEventListener !== 'function') {
      globalThis.self.addEventListener = () => {};
    }
    if (typeof globalThis.postMessage !== 'function') {
      globalThis.postMessage = () => {};
    }
    ({ RrfCommander } = await import('../../../examples/js/slideprinter/rrfCommander.js'));
    ({ FileFormat } = await import('../../../examples/js/slideprinter/fileFormatUtils.js'));
  });

  test('parses binary CAN movement entries into spool motion', async () => {
    const binaryCan = makeSingleEntryBinaryCan({
      time: 0,
      motorId: 40,
      steps: 5,
      accelTicks: 100,
    });

    const moves = await collectBinaryMoves(binaryCan);

    expect(moves.length).toBeGreaterThanOrEqual(1);
    expect(moves[0].A).toBeCloseTo(5 * STEP_ANGLE_RAD, 12);
  });

  test('handles uint64 timestamps without requiring BigInt APIs', async () => {
    const binaryCan = makeSingleEntryBinaryCan({
      time: 5_000_000_000,
      motorId: 40,
      steps: 3,
      accelTicks: 100,
    });

    const moves = await collectBinaryMoves(binaryCan, { dt: 10_000 });

    expect(moves.length).toBeGreaterThanOrEqual(1);
    expect(moves[0].A).toBeCloseTo(3 * STEP_ANGLE_RAD, 12);
  });
});
