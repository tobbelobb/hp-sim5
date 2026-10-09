import {
  World, PositionComponent, OrientationComponent, RadiusComponent, EncoderComponent,
  RigidBodyMemberComponent, SceneEntityInfoComponent, MachineTagComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { FlightRecorder } from '../../../hp-sim-3d/app/flightRecorder.js';
import { captureFlightRecorderSnapshot } from '../../../hp-sim-3d/app/flightRecorderSnapshot.js';

jest.mock('three/addons/controls/OrbitControls.js', () => ({
  OrbitControls: class OrbitControls {},
}));

function body(world, name, position, tags = []) {
  const id = world.createEntity();
  world.addComponent(id, new SceneEntityInfoComponent(name, tags));
  world.addComponent(id, new MachineTagComponent('hp4'));
  world.addComponent(id, new PositionComponent(...position));
  world.addComponent(id, new RadiusComponent(0.1));
  return id;
}

function drivenCable({ driveAtEnd = false, clockwise = true } = {}) {
  const world = new World();
  world.setResource('enableLayering', false);
  const a = body(world, 'SpoolA', [0, 0, 0]);
  const b = body(world, 'AnchorA', [1, 0, 0], ['Anchor']);
  const jointId = world.createEntity();
  const joint = CableJointComponent.fromWorld(a, b, 1, new Vector3(), new Vector3(1, 0, 0));
  world.addComponent(jointId, joint);
  const path = new CablePathComponent(
    world, [jointId], driveAtEnd ? ['attachment', 'hybrid'] : ['hybrid', 'attachment'],
    [clockwise, clockwise], 1e6, driveAtEnd ? [0, 0.5] : [0.5, 0],
  );
  const pathId = world.createEntity();
  world.addComponent(pathId, path);
  world.addComponent(pathId, new MachineTagComponent('hp4'));
  const endpoint = driveAtEnd ? b : a;
  const motor = new StepperMotorComponent(0.6, 0.1);
  world.addComponent(endpoint, motor);
  world.addComponent(endpoint, new EncoderComponent(0.2));
  return { world, joint, motor };
}

describe('flight recorder ECS snapshot', () => {
  test.each([
    [false, true, 0.97], [true, true, 1.03],
    [false, false, 1.03], [true, false, 0.97],
  ])('motor winding has the correct payout sign (end=%s, cw=%s)', (driveAtEnd, clockwise, commanded) => {
    const { world } = drivenCable({ driveAtEnd, clockwise });
    const { lengths } = captureFlightRecorderSnapshot(world).cables[0];
    expect(lengths.actual).toBe(1);
    expect(lengths.geometric).toBe(1);
    expect(lengths.commanded).toBeCloseTo(commanded, 12);
    expect(lengths.error).toBeCloseTo(1 - commanded, 12);
  });

  test('logs transferred cable force even without force overlays, and omits torque-mode targets', () => {
    const { world, joint, motor } = drivenCable();
    world.setResource('showConstraintForces', false);
    joint.constraintForceMagnitude = 3;
    joint.transferredConstraintForceMagnitude = 7;
    motor.torqueMode = true;
    const cable = captureFlightRecorderSnapshot(world).cables[0];
    expect(cable.segments[0].force_n).toBe(7);
    expect(cable.segments[0].force_vector_n).toEqual([7, 0, 0]);
    expect(cable.lengths.commanded).toBeNull();
    expect(cable.lengths.error).toBeNull();
  });

  test('captures slack with the same sag as the renderer and copies mutable physics data', () => {
    const { world, joint } = drivenCable();
    joint.restLength = 1.2;
    const snapshot = captureFlightRecorderSnapshot(world);
    const segment = snapshot.cables[0].segments[0];
    expect(segment.points[0]).toEqual([0, 0, 0]);
    expect(segment.points.at(-1)).toEqual([1, 0, 0]);
    expect(segment.points[8][2]).toBeLessThan(0);
    expect(snapshot.cables[0].lengths.stretch).toBeCloseTo(-0.2);
    joint.attachmentPointA_world.x = 99;
    expect(segment.points[0]).toEqual([0, 0, 0]);
  });

  test('compact geometry preserves lengths and forces while simplifying the drawing', () => {
    const { world, joint } = drivenCable();
    joint.restLength = 1.2;
    const full = captureFlightRecorderSnapshot(world).cables[0];
    const compact = captureFlightRecorderSnapshot(world, { geometryDetail: 'compact' }).cables[0];
    expect(compact.lengths).toEqual(full.lengths);
    expect(compact.segments[0].force_vector_n).toEqual(full.segments[0].force_vector_n);
    expect(compact.segments[0].points).toEqual([[0, 0, 0], [1, 0, 0]]);
    expect(full.segments[0].points.length).toBeGreaterThan(2);
    expect(compact.wraps).toEqual([]);
  });

  test('uses the final rigid-body pose for the effector and logs members in their parent frame', () => {
    const world = new World();
    const parent = body(world, 'EffectorBody', [10, 20, 30], ['RigidBody']);
    const rotation = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2);
    world.addComponent(parent, new OrientationComponent(rotation.x, rotation.y, rotation.z, rotation.w));
    // The member's PositionComponent can still contain the pre-solve sync pose.
    const member = body(world, 'AttachmentA', [100, 100, 100]);
    world.addComponent(member, new RigidBodyMemberComponent(parent, new Vector3(1, 0, 0), new Quaternion()));
    body(world, 'AnchorA', [0, 0, 2], ['Anchor']);
    const extruderId = world.createEntity();
    const extruder = new ExtruderComponent();
    extruder.centerSources.hp4 = [member];
    world.addComponent(extruderId, extruder);

    const frames = captureFlightRecorderSnapshot(world).frames;
    const effector = frames.find((frame) => frame.kind === 'effector');
    expect(effector.position[0]).toBeCloseTo(10);
    expect(effector.position[1]).toBeCloseTo(21);
    expect(effector.position[2]).toBeCloseTo(30);
    expect(effector.quaternion).toEqual([rotation.x, rotation.y, rotation.z, rotation.w]);
    const attachment = frames.find((frame) => frame.name === 'AttachmentA');
    expect(attachment.path).toContain('/EffectorBody_0/members/AttachmentA_1');
    expect(attachment.position).toEqual([1, 0, 0]);
    expect(frames.filter((frame) => frame.kind === 'anchor')).toHaveLength(1);
  });
});

class FakeSocket {
  constructor() {
    this.readyState = 0;
    this.listeners = new Map();
    this.samples = [];
  }
  addEventListener(type, listener) { this.listeners.set(type, listener); }
  emit(type, message) { this.listeners.get(type)?.({ data: JSON.stringify(message) }); }
  send(message) { this.samples.push(JSON.parse(message)); }
  close() { this.readyState = 3; }
}

describe('flight recorder delivery', () => {
  const cryptoDescriptor = Object.getOwnPropertyDescriptor(globalThis, 'crypto');
  beforeAll(() => {
    Object.defineProperty(globalThis, 'crypto', { configurable: true, value: { randomUUID: () => 'session' } });
  });
  afterAll(() => {
    if (cryptoDescriptor) Object.defineProperty(globalThis, 'crypto', cryptoDescriptor);
    else delete globalThis.crypto;
  });

  test('bounds pending samples, resumes on acknowledgement, and resets clocks on a scene change', () => {
    const world = new World();
    world.setResource('sceneGeneration', 1);
    const recorder = new FlightRecorder({ world, WebSocketClass: FakeSocket });
    recorder.connect();
    expect(recorder.readyForStep()).toBe(false);
    const socket = recorder.socket;
    socket.readyState = 1;
    socket.emit('open');
    while (recorder.readyForStep()) recorder.update(world, 0.001);
    expect(socket.samples).toHaveLength(32);
    expect(socket.samples.map((sample) => sample.step)).toEqual(Array.from({ length: 32 }, (_, i) => i));
    expect(socket.samples.at(-1).time).toBeCloseTo(0.031);
    socket.emit('message', { type: 'ack' });
    expect(recorder.readyForStep()).toBe(true);
    recorder.update(world, 0.001);
    expect(socket.samples.at(-1).step).toBe(32);

    world.setResource('sceneGeneration', 2);
    socket.emit('message', { type: 'ack' });
    recorder.update(world, 0.001);
    expect(socket.samples.at(-1)).toMatchObject({ generation: 2, step: 1, time: 0.001 });
    recorder.disconnect();
    expect(recorder.readyForStep()).toBe(true);
    socket.emit('message', { type: 'ack' });
    expect(recorder.pending).toBe(0);
  });

  test('sampling preserves actual step indices and both clocks; events remain unsampled', () => {
    const world = new World();
    world.setResource('sceneGeneration', 1);
    const recorder = new FlightRecorder({ world, WebSocketClass: FakeSocket });
    recorder.connect();
    const socket = recorder.socket;
    socket.readyState = 1; socket.emit('open');
    socket.emit('message', { type: 'recording_config', sample_stride: 10, geometry_detail: 'compact' });
    recorder.extendedReference = true;
    for (let step = 1; step <= 21; step++) {
      world.setResource('researchClock', { time: step * .001 });
      recorder.update(world, .001);
      recorder.recordEvent('test', { step });
    }
    const samples = socket.samples.filter(sample => sample.type !== 'autocal_event');
    expect(samples.map(sample => sample.step)).toEqual([0, 10, 20]);
    expect(samples[2].time).toBeCloseTo(.020);
    expect(samples[2].research_clock.time).toBe(.020);
    expect(samples[2].sample_stride).toBe(10);
    expect(socket.samples.filter(sample => sample.type === 'autocal_event')).toHaveLength(21);
  });
});

test('extended browser events preserve both clocks and do not release physics backpressure', () => {
  const world = new World();
  world.setResource('sceneGeneration', 2);
  world.setResource('researchClock', { generation: 2, step: 15, time: 1.5 });
  const recorder = new FlightRecorder({ world, WebSocketClass: FakeSocket });
  recorder.socket = new FakeSocket();
  recorder.socket.readyState = 1;
  recorder.extendedReference = true;
  recorder.time = .5;
  recorder.pending = 32;
  recorder.recordEvent('external_payload_received', { type: 'encoder_request', requestId: 4 });
  expect(recorder.socket.samples[0]).toMatchObject({
    source: 'browser', sim_time_s: 1.5, sim_time_source: 'browser.researchClock',
    recorder_sim_time_s: .5, generation: 2,
    payload: { requestId: 4 },
  });
  expect(Number.isInteger(recorder.socket.samples[0].wall_time_ms)).toBe(true);
  expect(recorder.readyForStep()).toBe(false);
});
