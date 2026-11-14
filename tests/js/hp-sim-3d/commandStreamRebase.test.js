import { readFileSync } from 'node:fs';
import { createHeadlessWorld } from '../../../hp-sim-3d/app/headlessWorld.js';
import { createExternalCommandController } from '../../../hp-sim-3d/app/externalCommandSocket.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

jest.mock('three/addons/controls/OrbitControls.js', () => ({ OrbitControls: class {} }));

class Socket extends EventTarget {
  static OPEN = 1;
  readyState = 0;
  constructor(url) { super(); this.url = url; Socket.last = this; }
  send() {}
  close() { this.readyState = 3; this.dispatchEvent(new Event('close')); }
  open() { this.readyState = 1; this.dispatchEvent(new Event('open')); }
}

test('each bridge connection maps its first motor angles onto the current targets', () => {
  jest.useFakeTimers();
  try {
    const { world, remote, dt } = createHeadlessWorld(readFileSync('public/usd_scenes/hp3.usda', 'utf8'));
    const controller = createExternalCommandController({
      world, url: 'ws://bridge', WebSocketCtor: Socket, logger: { info() {}, log() {}, warn() {} },
      commands: { getRemoteSystem: () => remote, pushExternalCommands(batch) { batch.forEach(c => remote.addCommand(c)); return true; } },
    });
    const drain = () => { while (remote.getQueueLength() > 0) world.update(dt); };
    remote._ensureAxisMapping(world);
    const stepper = world.getComponent(remote.axisToEntity.B[0], StepperMotorComponent);
    const target = () => stepper.commandedAngle - stepper.deltaAngle;
    const start = target();

    controller.connect();
    Socket.last.open();
    controller.handlePayload({ type: 'command', command: { type: 'Move', B: 0 } });
    controller.handlePayload({ type: 'command', command: { type: 'Move', B: 25 } });
    drain();
    expect(target()).toBeCloseTo(start + 25, 12);

    // The next collector's bridge counts from zero again: no jump back by 25 rad.
    Socket.last.close();
    jest.runOnlyPendingTimers();
    Socket.last.open();
    controller.handlePayload({ type: 'command', command: { type: 'Move', B: 0 } });
    drain();
    expect(target()).toBeCloseTo(start + 25, 12);
    controller.handlePayload({ type: 'command', command: { type: 'Move', B: -3 } });
    drain();
    expect(target()).toBeCloseTo(start + 22, 12);
  } finally {
    jest.useRealTimers();
  }
});
