import { createGcodeBridge } from '../../../integrations/rrf/rrfSimulatorBridge.mjs';
import { KlippyApiClient } from '../../../integrations/klipper/klippyApiClient.js';
import { KlippyRuntimeState } from '../../../integrations/klipper/klippyRuntimeState.js';
import { createKlipperTerminalBridge } from '../../../integrations/klipper/klipperTerminalBridge.js';
import {
  DEFAULT_KLIPPY_CONFIG_PATH,
  DEFAULT_KLIPPY_SOCKET_PATH,
} from '../../../integrations/klipper/klippy_api_cli_config.mjs';

export async function createBridge(firmware, options = {}) {
  if (firmware !== 'klipper') {
    return createGcodeBridge(options);
  }

  const socketPath = options.socketPath || DEFAULT_KLIPPY_SOCKET_PATH;
  const configPath = options.configPath || DEFAULT_KLIPPY_CONFIG_PATH;
  const speedScale = Number.isFinite(options.speedup) && options.speedup > 0 ? options.speedup : 1;
  const motionIdleMs = Number.isFinite(options.motionIdleMs)
    ? options.motionIdleMs
    : options.sim
      ? Math.max(40, Math.round(650 / speedScale))
      : 650;

  const client = new KlippyApiClient({ socketPath });
  const klippyState = new KlippyRuntimeState({ client });
  const klipperBridge = createKlipperTerminalBridge({
    client,
    klippyState,
    wsPort: options.wsPort,
    quiet: options.quiet,
    configPath,
    encoderTimeoutMs: options.encoderTimeoutMs,
    motionIdleMs,
  });

  const onGcodeOutput = ({ response }) => {
    klipperBridge.handleGcodeOutput(response);
  };
  klippyState.on('gcode-output', onGcodeOutput);

  try {
    client.start();
    await client.waitForConnection();
    await klippyState.prime();
    await klippyState.waitForReady();
  } catch (error) {
    klippyState.off('gcode-output', onGcodeOutput);
    try {
      klipperBridge.close();
    } catch (_error) {
      // ignore cleanup errors
    }
    client.close();
    throw error;
  }

  const sendGcodeLine = async (line) => {
    const trimmed = line?.trim?.();
    if (!trimmed) {
      return null;
    }
    await client.waitForConnection();
    await klippyState.prime();
    await klippyState.waitForReady();
    const rewritten = typeof klipperBridge.rewriteGcodeLine === 'function'
      ? klipperBridge.rewriteGcodeLine(trimmed)
      : trimmed;
    return klipperBridge.runGcodeCommand(
      trimmed,
      () => client.request('gcode/script', { script: rewritten }),
    );
  };

  const close = () => {
    klippyState.off('gcode-output', onGcodeOutput);
    klipperBridge.close();
    client.close();
  };

  return {
    ...klipperBridge,
    client,
    klippyState,
    sendGcodeLine,
    close,
  };
}
