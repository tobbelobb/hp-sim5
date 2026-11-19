// Harness around the two SHIPPED playback paths, unmodified source.
//
//  * preset path  : integrations/rrf/rrfCanPlayer.js  (class RrfCanPlayer)
//  * upload path  : hp-sim-3d/app/workers/moveCommander.js (class MoveCommander)
//
// Both are instantiated exactly as the app's worker does, with the only
// difference being that sendCommand() is captured in-process instead of
// postMessage()d to the world, and the wall-clock pacing is disabled.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { performance } from 'node:perf_hooks';

globalThis.self = { addEventListener() {} };
globalThis.postMessage = (message) => {
  if (message && message.type === 'error') {
    throw new Error(message.message);
  }
};

const ROOT = new URL('../../../../../', import.meta.url); // repo root
const { FileFormat } = await import(new URL('integrations/shared/fileFormatUtils.js', ROOT));
const { RrfCanPlayer } = await import(new URL('integrations/rrf/rrfCanPlayer.js', ROOT));
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));
const { parseRrfMotorAxisMapFromConfigText } = await import(
  new URL('integrations/rrf/rrfFirmwareModel.js', ROOT));

export { RrfCanPlayer, MoveCommander, FileFormat, parseRrfMotorAxisMapFromConfigText };

/**
 * Decode a shipped .can plan with the app's own player.
 * Returns the sequence of commands the world would receive.
 */
export async function playCan(canPath, configText, onCommand = null) {
  const player = new RrfCanPlayer();
  if (configText) {
    const map = parseRrfMotorAxisMapFromConfigText(configText);
    player.setDriverToAxis(Array.from(map.entries()));
  }
  player.setAsapMode(true);          // no wall-clock waits
  player._targetWaitMs = () => 0;
  const commands = [];
  player.sendCommand = async (command) => {
    if (onCommand) onCommand(command);
    else commands.push(structuredClone(command));
  };
  const started = performance.now();
  await player.run(
    Readable.toWeb(fs.createReadStream(canPath)),
    FileFormat.RRF_CAN_BINARY,
  );
  return { commands, wallMs: performance.now() - started };
}

/** Run a G-code text through the upload path (MoveCommander) with a machine config. */
export async function playUpload(gcodeText, configText, onCommand = null) {
  const commander = new MoveCommander();
  commander.configure(configText);
  commander.setAsapMode(true);       // no wall-clock waits
  const commands = [];
  commander.sendCommand = async (command) => {
    if (onCommand) onCommand(command);
    else commands.push(structuredClone(command));
  };
  const started = performance.now();
  await commander.run(new ReadableStream({
    start(controller) {
      controller.enqueue(new TextEncoder().encode(gcodeText));
      controller.close();
    },
  }));
  return { commands, wallMs: performance.now() - started, commander };
}

/** Fold a command stream into per-axis absolute angle series. */
export function foldAngles(commands, axes) {
  const offsets = Object.fromEntries(axes.map((a) => [a, 0]));
  const out = [];
  for (const c of commands) {
    if (c.type === 'Add to reference') {
      for (const a of axes) if (Number.isFinite(c[a])) offsets[a] += c[a];
      continue;
    }
    if (c.type !== 'Move') continue;
    const angles = {};
    for (const a of axes) angles[a] = (Number.isFinite(c[a]) ? c[a] : 0) + offsets[a];
    out.push({ angles, E: Number.isFinite(c.E) ? c.E : 0 });
  }
  return out;
}
