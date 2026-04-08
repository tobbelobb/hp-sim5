import fs from 'node:fs/promises';
import path from 'node:path';

export const FAKE_GPIO_HELPER_SCRIPT = './scripts/make-fake-pin-chip.sh';

export function isMissingFakeGpioChipStateMessage(stateMessage) {
  return typeof stateMessage === 'string'
    && stateMessage.includes('GPIO chip device not found');
}

export function buildFakeGpioChipSetupCommand(configPath, cwd = process.cwd()) {
  return {
    command: 'sudo',
    args: [
      path.resolve(cwd, FAKE_GPIO_HELPER_SCRIPT),
      path.resolve(cwd, configPath),
    ],
  };
}

export async function configNeedsGpioChipSetup(configPath) {
  try {
    const source = await fs.readFile(configPath, 'utf8');
    return /gpiochip(?:REPLACE_ME_WITH_FAKE_GPIO_CHIP_NUMBER|\d+)\/gpio\d+/u.test(source);
  } catch (_error) {
    return false;
  }
}
