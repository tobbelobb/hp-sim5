import assert from 'node:assert/strict';
import { execFileSync, spawn } from 'node:child_process';
import { once } from 'node:events';
import { createReadStream } from 'node:fs';
import { mkdir, mkdtemp, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { Readable } from 'node:stream';
import { setTimeout as delay } from 'node:timers/promises';
import { fileURLToPath } from 'node:url';

import { RrfHttpBridge } from '../../integrations/rrf/rrfHttpBridge.js';
import { parseRrfMotorAxisMapFromConfigText, STEP_ANGLE_RAD } from '../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../integrations/shared/fileFormatUtils.js';

globalThis.self = { addEventListener() {} };
globalThis.postMessage = (message) => {
    if (message.type === 'error') throw new Error(message.message);
};
const { RrfCanPlayer } = await import('../../integrations/rrf/rrfCanPlayer.js');

const repo = fileURLToPath(new URL('../..', import.meta.url));
const simulator = join(repo, 'RRF/build/rrf_simulator');
const python = join(repo, '.venv/bin/python');
const port = process.env.PORT || 8093;
const config = `
G21
G90
M83
M584 X40.0 Y40.1 Z40.2 E40.3 P3
M669 K0
M92 X80 Y80 Z400 E95.922
M201 X3000 Y3000 Z3000 E3000
M203 X18000 Y18000 Z18000 E18000
M204 P3000 T3000
M566 X300 Y300 Z300 E1200
M569 P40.0 S1
M569 P40.1 S1
M569 P40.2 S1
M569 P40.3 S1
M564 H0 S0
M563 P0 D0
T0
M302 P1
`;
const origin = 'G92 X0 Y0 Z0';
const xyMove = 'G1 X10 Y5 F600';
const zExtrusionMove = 'G1 Z1 E2 F600';
const driverToAxis = parseRrfMotorAxisMapFromConfigText(config);

function near(actual, expected, description, tolerance = 1e-5) {
    assert.ok(Math.abs(actual - expected) < tolerance, `${description}: got ${actual}, expected ${expected}`);
}

function checkMoves(commands, zSteps = 0, extrusion = 0) {
    const moves = commands.filter(command => command.type === 'Move');
    assert.ok(moves.length > 10, 'the shaped move should span multiple playback buckets');
    const final = moves.at(-1);
    near(final.A / STEP_ANGLE_RAD, 800, 'X steps');
    near(final.B / STEP_ANGLE_RAD, 400, 'Y steps');
    near((final.C ?? 0) / STEP_ANGLE_RAD, zSteps, 'Z steps');
    near(moves.reduce((sum, command) => sum + (command.E ?? 0), 0), extrusion, 'total extrusion in mm');
    // The first bucket ends at tick 1500, during the firmware's acceleration ramp.
    near(moves[0].A / STEP_ANGLE_RAD, 0.429325, 'first X bucket in steps', 0.002);
    return moves;
}

async function playCapture(filename, format, mapping) {
    const player = new RrfCanPlayer();
    player.setDriverToAxis(mapping);
    player.setAsapMode(true);
    const commands = [];
    player.sendCommand = async command => commands.push(command);
    await player.run(Readable.toWeb(createReadStream(filename)), format);
    return commands;
}

const vsd = await mkdtemp(join(tmpdir(), 'rrf-multidriver-'));
let server;
try {
    await mkdir(join(vsd, 'sys'));
    await mkdir(join(vsd, 'gcodes'));
    await writeFile(join(vsd, 'sys/config.g'), config);
    const args = ['--vsd', vsd, '-c', 'sys/config.g'];
    server = spawn(simulator, [...args, '--server', '-p', port], { stdio: 'ignore' });
    const spoolCommands = [];
    const bridge = new RrfHttpBridge({
        baseUrl: `http://127.0.0.1:${port}`,
        driverToAxis,
        remoteSpoolSystem: { addCommand: command => spoolCommands.push(command) },
    });
    for (let attempt = 0; ; attempt += 1) {
        try {
            const response = await bridge.sendGCode('M115', { timeout: 500 });
            assert.match(response.reply, /FIRMWARE_NAME/);
            break;
        } catch (error) {
            if (attempt === 49 || server.exitCode !== null || server.signalCode !== null) throw error;
            await delay(100);
        }
    }
    await bridge.sendGCode(origin);
    const move = await bridge.sendGCode(xyMove);
    assert.deepEqual(move.motion.map(command => [command.canAddress, command.driver, command.steps]), [
        [40, 0, 800],
        [40, 1, 400],
    ]);
    near(move.motion[0].acceleration, 4.77027862e-10, 'firmware profile', 1e-16);
    checkMoves(spoolCommands);
    const mixedMove = await bridge.sendGCode(zExtrusionMove);
    assert.deepEqual(mixedMove.motion.map(command => [command.driver, command.isExtruder]), [
        [0, false], [1, false], [2, false], [3, true],
    ]);
    near(mixedMove.motion[2].steps, 400, 'Z steps after the empty driver slots');
    checkMoves(spoolCommands, 400, 2);

    // Also exercise the full seven-bit address in a batch capture.
    const highAddressConfig = config.replaceAll('40.', '104.');
    await writeFile(join(vsd, 'sys/config-recording.g'), highAddressConfig);
    await writeFile(join(vsd, 'gcodes/probe.gcode'), `${origin}\n${xyMove}\n${zExtrusionMove}\n`);
    const csvPath = join(vsd, 'probe.csv');
    execFileSync(simulator, ['--vsd', vsd, '-c', 'sys/config-recording.g',
        '--gcode', 'gcodes/probe.gcode', '--can-log', csvPath], { timeout: 15000 });
    execFileSync(python, [join(repo, 'RRF/csv2can.py'), csvPath], { timeout: 5000 });

    const mapping = parseRrfMotorAxisMapFromConfigText(highAddressConfig);
    const csvMoves = checkMoves(await playCapture(csvPath, FileFormat.RRF_CAN, mapping), 400, 2);
    const binaryMoves = checkMoves(await playCapture(join(vsd, 'probe.can'), FileFormat.RRF_CAN_BINARY, mapping), 400, 2);
    assert.equal(binaryMoves.length, csvMoves.length, 'binary playback retains the CSV bucket cadence');
    for (let i = 0; i < csvMoves.length; i += 1) {
        for (const axis of ['A', 'B', 'C', 'E']) {
            near(binaryMoves[i][axis] ?? 0, csvMoves[i][axis] ?? 0, `bucket ${i} axis ${axis}`);
        }
    }
    console.log('PASS: RRF multi-driver HTTP, CSV, and binary playback');
} finally {
    if (server && server.exitCode === null && server.signalCode === null) {
        const exited = once(server, 'exit');
        server.kill();
        await exited;
    }
    await rm(vsd, { recursive: true, force: true });
}
