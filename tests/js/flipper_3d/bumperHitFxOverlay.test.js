import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  World,
  PositionComponent,
  RadiusComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import { RenderableComponent } from '../../../src/js/cable_joints/ecs.js';
import { ObstaclePushComponent } from '../../../examples/js/flipper/flipper_common.js';
import { BumperHitFxOverlay } from '../../../examples/js/flipper_3d/bumperHitFxOverlay.js';

function createMockCanvas() {
  const gradient = { addColorStop: jest.fn() };
  const ctx = {
    clearRect: jest.fn(),
    createRadialGradient: jest.fn(() => gradient),
    fillRect: jest.fn(),
    beginPath: jest.fn(),
    moveTo: jest.fn(),
    lineTo: jest.fn(),
    stroke: jest.fn(),
    _fillStyle: '',
    _strokeStyle: '',
    set fillStyle(value) {
      this._fillStyle = value;
    },
    get fillStyle() {
      return this._fillStyle;
    },
    set strokeStyle(value) {
      this._strokeStyle = value;
    },
    get strokeStyle() {
      return this._strokeStyle;
    },
    lineWidth: 0
  };
  const canvas = {
    width: 160,
    height: 270,
    getBoundingClientRect: () => ({ width: 160, height: 270 }),
    getContext: () => ctx
  };
  return { canvas, ctx };
}

describe('BumperHitFxOverlay', () => {
  test('spawns bursts while the feature flag is enabled', () => {
    const world = new World();
    const ball = world.createEntity();
    world.addComponent(ball, new PositionComponent(0.1, 0.1, 0.0));
    world.addComponent(ball, new RadiusComponent(0.03));

    const obstacle = world.createEntity();
    world.addComponent(obstacle, new PositionComponent(0.35, 0.2, 0.0));
    world.addComponent(obstacle, new RadiusComponent(0.06));
    world.addComponent(obstacle, new RenderableComponent('circle', '#ffaa00'));
    world.addComponent(obstacle, new ObstaclePushComponent(1.2));

    world.setResource('renderBumperHitFx', true);
    world.setResource('simWidth', 1.0);
    world.setResource('simHeight', 1.7);
    world.setResource('ball_obstacle_contacts', [{
      ball_id: ball,
      obs_id: obstacle,
      raw_hit: true,
      delta_lambda: 0.4,
      direction: new Vector3(1.0, 0.0, 0.0)
    }]);

    const { canvas, ctx } = createMockCanvas();
    const overlay = new BumperHitFxOverlay(world, canvas);
    overlay.update(world, null);

    expect(overlay.bursts.length).toBeGreaterThanOrEqual(1);
    expect(ctx.clearRect).toHaveBeenCalled();
    expect(ctx.fillRect).toHaveBeenCalled();
  });

  test('clears bursts when the feature flag is disabled', () => {
    const world = new World();
    const ball = world.createEntity();
    world.addComponent(ball, new PositionComponent(0.1, 0.1, 0.0));
    world.addComponent(ball, new RadiusComponent(0.03));

    const obstacle = world.createEntity();
    world.addComponent(obstacle, new PositionComponent(0.5, 0.25, 0.0));
    world.addComponent(obstacle, new RadiusComponent(0.05));
    world.addComponent(obstacle, new RenderableComponent('circle', '#ff9000'));
    world.addComponent(obstacle, new ObstaclePushComponent(1.8));

    world.setResource('simWidth', 1.0);
    world.setResource('simHeight', 1.7);
    world.setResource('ball_obstacle_contacts', [{
      ball_id: ball,
      obs_id: obstacle,
      raw_hit: true,
      delta_lambda: 0.2,
      direction: new Vector3(0.0, 1.0, 0.0)
    }]);
    const { canvas } = createMockCanvas();
    const overlay = new BumperHitFxOverlay(world, canvas);

    world.setResource('renderBumperHitFx', true);
    overlay.update(world, null);
    expect(overlay.bursts.length).toBeGreaterThanOrEqual(1);

    world.setResource('renderBumperHitFx', false);
    overlay.update(world, null);
    expect(overlay.bursts.length).toBe(0);
  });

  test('does not spawn for contacts without raw hits', () => {
    const world = new World();
    const ball = world.createEntity();
    world.addComponent(ball, new PositionComponent(0.1, 0.1, 0.0));
    world.addComponent(ball, new RadiusComponent(0.03));

    const obstacle = world.createEntity();
    world.addComponent(obstacle, new PositionComponent(0.5, 0.25, 0.0));
    world.addComponent(obstacle, new RadiusComponent(0.05));
    world.addComponent(obstacle, new RenderableComponent('circle', '#ff9000'));
    world.addComponent(obstacle, new ObstaclePushComponent(1.8));

    world.setResource('simWidth', 1.0);
    world.setResource('simHeight', 1.7);
    world.setResource('ball_obstacle_contacts', [{
      ball_id: ball,
      obs_id: obstacle,
      raw_hit: false,
      delta_lambda: 0.2,
      direction: new Vector3(0.0, 1.0, 0.0)
    }]);
    const { canvas, ctx } = createMockCanvas();
    const overlay = new BumperHitFxOverlay(world, canvas);

    world.setResource('renderBumperHitFx', true);
    overlay.update(world, null);

    expect(overlay.bursts.length).toBe(0);
    expect(ctx.clearRect).toHaveBeenCalled();
  });

});
