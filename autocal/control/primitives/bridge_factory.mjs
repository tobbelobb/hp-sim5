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
