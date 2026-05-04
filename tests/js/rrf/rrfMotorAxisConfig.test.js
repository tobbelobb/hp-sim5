import { RrfHttpBridge } from '../../../integrations/rrf/rrfHttpBridge.js';
import {
  STEP_ANGLE_RAD,
  parseRrfMotorAxisMapFromConfigText,
} from '../../../integrations/rrf/rrfFirmwareModel.js';

const CUBECORNERS_M584 = 'M584 X40.0 Y41.0 Z42.0 U43.0 V44.0 W45.0 A46.0 B47.0 E48.0 P8';

describe('RRF motor axis config', () => {
  test('maps RRF visible axes to hp-sim spool axes from M584', () => {
    const driverToAxis = parseRrfMotorAxisMapFromConfigText(CUBECORNERS_M584);

    expect(driverToAxis.get(40)).toBe('A');
    expect(driverToAxis.get(43)).toBe('D');
    expect(driverToAxis.get(44)).toBe('I');
    expect(driverToAxis.get(45)).toBe('J');
    expect(driverToAxis.get(46)).toBe('L');
    expect(driverToAxis.get(47)).toBe('O');
    expect(driverToAxis.get(48)).toBe('E');
  });

  test('uses parsed config mapping when streaming HTTP motion', async () => {
    const commands = [];
    const driverToAxis = parseRrfMotorAxisMapFromConfigText(CUBECORNERS_M584);
    const bridge = new RrfHttpBridge({
      driverToAxis,
      remoteSpoolSystem: {
        addCommand(command) {
          commands.push(command);
        },
      },
      fetchImpl: async () => ({
        ok: true,
        text: async () => [
          'ok',
          '---MOTION---',
          '0,44,0,100,0,0,5,0,0',
        ].join('\n'),
      }),
    });

    await bridge.sendGCode('G1 H2 V10 F5000');

    const move = commands.find((command) => command.type === 'Move');
    expect(move.I).toBeCloseTo(5 * STEP_ANGLE_RAD, 12);
    expect(move.E).toBeUndefined();
  });
});
