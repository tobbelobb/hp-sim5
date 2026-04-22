jest.mock('../../../src/js/cable_joints_3d/render_system_3d.js', () => {
  class MockRenderSystem3D {
    constructor(canvas, options) {
      this.canvas = canvas;
      this.options = options;
      this.setViewTransform = jest.fn();
    }

    resetVisuals() {}
    setCanvasSize() {}
    update() {}
  }

  return { RenderSystem3D: MockRenderSystem3D };
});

jest.mock('../../../src/js/usd/stage.js', () => ({
  getAttribute: jest.fn((prim, attr) => {
    if (prim?.path === '/World/PhysicsScene') {
      if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
      if (attr === 'physics:gravityMagnitude') return 9.82;
    }
    if (prim?.path === '/World/SlideprinterScene/SpoolA') {
      if (attr === 'ecs:tags') return ['Spool'];
      if (attr === 'xformOp:translate') return [0.2, 0.3, 0.0];
      if (attr === 'radius') return 0.05;
      if (attr === 'physics:mass') return 1.0;
      if (attr === 'physics:inertiaTensor') return [[0, 0, 0], [0, 0, 0], [0, 0, 0.01]];
      if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
      if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
      if (attr === 'stepper:holdingTorque') return 0.7;
      if (attr === 'stepper:numPolePairs') return 50;
      if (attr === 'stepper:dampingCoeff') return 0.01;
      if (attr === 'stepper:maxSpeedRad') return 600;
      if (attr === 'cable:linkable') return true;
    }
    return null;
  }),
  getChild: jest.fn(() => null),
  getChildren: jest.fn((prim) => {
    if (prim?.path === '/World/SlideprinterScene') {
      return [{
        path: '/World/SlideprinterScene/SpoolA',
        name: 'SpoolA',
        type: 'definition',
        defType: 'Sphere'
      }];
    }
    return [];
  }),
  getRelationship: jest.fn(() => []),
  materialProperties: jest.fn(() => ({ color: '#8899aa', friction: null, restitution: null }))
}));

const { World } = require('../../../src/js/cable_joints_3d/ecs.js');
const { setupScene } = require('../../../hp-sim-3d/app/setupScene.js');
const { RenderSystem3D } = require('../../../src/js/cable_joints_3d/render_system_3d.js');
const {
  RenderableComponent,
  EncoderComponent,
  OrientationComponent,
  AngularVelocityComponent,
  RadiusComponent,
  PositionComponent,
} = require('../../../src/js/cable_joints_3d/ecs.js');
const { CablePathComponent } = require('../../../src/js/cable_joints_3d/cable_joints_core.js');
const { CableLinkComponent } = require('../../../src/js/cable_joints_3d/cable_joints_core.js');
const { ExtruderComponent } = require('../../../hp-sim-3d/app/hangprinter_extruder.js');
const { PauseStateComponent } = require('../../../example_apps/js/flipper/flipper_common.js');
const { SpoolTagComponent, SpoolStateComponent } = require('../../../hp-sim-3d/app/hangprinter_spools.js');
const { StepperMotorComponent } = require('../../../hp-sim-3d/app/hangprinter_stepper_motor.js');
const Quaternion = require('../../../src/js/cable_joints_3d/quaternion.js').default;
const Vector3 = require('../../../src/js/cable_joints_3d/vector3.js').default;
const usdStage = require('../../../src/js/usd/stage.js');

function installDefaultUsdStageMocks() {
  usdStage.getAttribute.mockImplementation((prim, attr) => {
    if (prim?.path === '/World/PhysicsScene') {
      if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
      if (attr === 'physics:gravityMagnitude') return 9.82;
    }
    if (prim?.path === '/World/SlideprinterScene/SpoolA') {
      if (attr === 'ecs:tags') return ['Spool'];
      if (attr === 'xformOp:translate') return [0.2, 0.3, 0.0];
      if (attr === 'radius') return 0.05;
      if (attr === 'physics:mass') return 1.0;
      if (attr === 'physics:inertiaTensor') return [[0, 0, 0], [0, 0, 0], [0, 0, 0.01]];
      if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
      if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
      if (attr === 'stepper:holdingTorque') return 0.7;
      if (attr === 'stepper:numPolePairs') return 50;
      if (attr === 'stepper:dampingCoeff') return 0.01;
      if (attr === 'stepper:maxSpeedRad') return 600;
      if (attr === 'cable:linkable') return true;
    }
    return null;
  });
  usdStage.getChildren.mockImplementation((prim) => {
    if (prim?.path === '/World/SlideprinterScene') {
      return [{
        path: '/World/SlideprinterScene/SpoolA',
        name: 'SpoolA',
        type: 'definition',
        defType: 'Sphere'
      }];
    }
    return [];
  });
  usdStage.getRelationship.mockImplementation(() => []);
}

function createCanvas() {
  return {
    clientWidth: 640,
    clientHeight: 960,
    width: 0,
    height: 0,
    style: {},
    setAttribute() {},
    focus() {},
    addEventListener() {},
    removeEventListener() {},
    getBoundingClientRect() {
      return {
        left: 0,
        top: 0,
        width: this.clientWidth,
        height: this.clientHeight
      };
    }
  };
}

describe('slideprinter 3D setupScene', () => {
  const originalDocument = global.document;
  const originalWindow = global.window;

  beforeEach(() => {
    installDefaultUsdStageMocks();
    global.document = {
      getElementById() {
        return { textContent: 'Pause' };
      },
      addEventListener() {},
      removeEventListener() {},
      body: { style: {} },
      documentElement: { style: {} }
    };
    global.window = {
      addEventListener() {},
      removeEventListener() {}
    };
  });

  afterEach(() => {
    global.document = originalDocument;
    global.window = originalWindow;
  });

  test('uses the 3D render system and registers 3D constraint systems', () => {
    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    setupScene(world, stage, createCanvas());

    const renderSystem = world.getResource('renderSystem');
    expect(renderSystem).toBeInstanceOf(RenderSystem3D);
    expect(renderSystem.options.targetX).toBe(0.0);
    expect(renderSystem.options.targetY).toBe(0.0);
    expect(renderSystem.options.initialOrbitAzimuth).toBeCloseTo(-Math.PI * 0.25, 6);
    expect(renderSystem.options.initialOrbitPolar).toBeCloseTo(1.05, 6);
    expect(renderSystem.setViewTransform).toHaveBeenCalledWith({
      scaleMultiplier: 1.1,
      offsetX: 0.0,
      offsetY: -0.0,
    });

    const planeNormal = world.getResource('defaultPlaneNormal');
    expect(planeNormal?.x).toBeCloseTo(0.0, 6);
    expect(planeNormal?.y).toBeCloseTo(0.0, 6);
    expect(planeNormal?.z).toBeCloseTo(1.0, 6);

    const systemNames = world.systems.map((system) => system.constructor.name);
    expect(systemNames).toContain('RigidBodySyncSystem');
    expect(systemNames).toContain('ExtruderSystem');
    expect(systemNames).toContain('EncoderUpdateSystem');
    expect(systemNames).toContain('StepperMotorSystem');
    expect(systemNames.filter((name) => name === 'RigidBodySyncSystem')).toHaveLength(1);
    expect(systemNames).not.toContain('SpoolAxisConstraintSystem');
  });

  test('preserves the current pause state when rebuilding the base scene', () => {
    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    world.setResource('pauseState', new PauseStateComponent(true));

    setupScene(world, stage, createCanvas(), { append: false });

    expect(world.getResource('pauseState')).toEqual(expect.objectContaining({ paused: true }));
  });

  test('reads authored cable path damping into CablePathComponent', () => {
    usdStage.getChildren.mockImplementation((prim) => {
      if (prim?.path === '/World/SlideprinterScene') {
        return [
          {
            path: '/World/SlideprinterScene/AnchorA',
            name: 'AnchorA',
            type: 'definition',
            defType: 'Sphere'
          },
          {
            path: '/World/SlideprinterScene/SpoolA',
            name: 'SpoolA',
            type: 'definition',
            defType: 'Sphere'
          },
          {
            path: '/World/SlideprinterScene/Joint1',
            name: 'Joint1',
            type: 'definition',
            defType: 'CableJoint'
          },
          {
            path: '/World/SlideprinterScene/CablePath1',
            name: 'CablePath1',
            type: 'definition',
            defType: 'Xform'
          }
        ];
      }
      return [];
    });

    usdStage.getAttribute.mockImplementation((prim, attr) => {
      if (prim?.path === '/World/PhysicsScene') {
        if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
        if (attr === 'physics:gravityMagnitude') return 9.82;
      }
      if (prim?.path === '/World/SlideprinterScene/AnchorA') {
        if (attr === 'xformOp:translate') return [0.0, 0.0, 0.0];
        if (attr === 'radius') return 0.05;
        if (attr === 'physics:mass') return -1.0;
      }
      if (prim?.path === '/World/SlideprinterScene/SpoolA') {
        if (attr === 'ecs:tags') return ['Spool'];
        if (attr === 'xformOp:translate') return [0.2, 0.0, 0.0];
        if (attr === 'radius') return 0.05;
        if (attr === 'physics:mass') return 1.0;
        if (attr === 'physics:inertiaTensor') return [[0, 0, 0], [0, 0, 0], [0, 0, 0.01]];
        if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
        if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
        if (attr === 'cable:linkable') return true;
      }
      if (prim?.path === '/World/SlideprinterScene/Joint1') {
        if (attr === 'restLength') return 0.2;
        if (attr === 'localPos0') return [0.0, 0.0, 0.0];
        if (attr === 'localPos1') return [0.0, 0.05, 0.0];
      }
      if (prim?.path === '/World/SlideprinterScene/CablePath1') {
        if (attr === 'apiSchemas') return ['CablePathAPI'];
        if (attr === 'cablePath:linkTypes') return ['attachment', 'attachment'];
        if (attr === 'cablePath:clockwise') return [true, true];
        if (attr === 'cablePath:stored') return [0.0, 0.0];
        if (attr === 'stiffness') return 1000.0;
        if (attr === 'cablePath:halfWidth') return 0.001;
        if (attr === 'cablePath:damping') return 0.125;
      }
      return null;
    });

    usdStage.getRelationship.mockImplementation((prim, rel) => {
      if (prim?.path === '/World/SlideprinterScene/Joint1' && rel === 'physics:body0') {
        return ['/World/SlideprinterScene/AnchorA'];
      }
      if (prim?.path === '/World/SlideprinterScene/Joint1' && rel === 'physics:body1') {
        return ['/World/SlideprinterScene/SpoolA'];
      }
      if (prim?.path === '/World/SlideprinterScene/CablePath1' && rel === 'cablePath:joints') {
        return ['/World/SlideprinterScene/Joint1'];
      }
      return [];
    });

    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    setupScene(world, stage, createCanvas());

    const pathEntities = world.query([CablePathComponent]);
    expect(pathEntities).toHaveLength(1);
    expect(world.getComponent(pathEntities[0], CablePathComponent).damping).toBeCloseTo(0.125, 12);
  });

  test('derives extruder offset from authored Extruder prim position and center sources', () => {
    usdStage.getChildren.mockImplementation((prim) => {
      if (prim?.path === '/World/SlideprinterScene') {
        return [
          {
            path: '/World/SlideprinterScene/SpoolA',
            name: 'SpoolA',
            type: 'definition',
            defType: 'Sphere'
          },
          {
            path: '/World/SlideprinterScene/AttachA',
            name: 'AttachA',
            type: 'definition',
            defType: 'Circle'
          },
          {
            path: '/World/SlideprinterScene/Extruder',
            name: 'Extruder',
            type: 'definition',
            defType: 'Sphere'
          }
        ];
      }
      return [];
    });
    usdStage.getAttribute.mockImplementation((prim, attr) => {
      if (prim?.path === '/World/PhysicsScene') {
        if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
        if (attr === 'physics:gravityMagnitude') return 9.82;
      }
      if (prim?.path === '/World/SlideprinterScene/SpoolA') {
        if (attr === 'ecs:tags') return ['Spool'];
        if (attr === 'xformOp:translate') return [0.2, 0.3, 0.0];
        if (attr === 'radius') return 0.05;
        if (attr === 'physics:mass') return 1.0;
        if (attr === 'physics:inertiaTensor') return [[0, 0, 0], [0, 0, 0], [0, 0, 0.01]];
        if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
        if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
        if (attr === 'stepper:holdingTorque') return 0.7;
        if (attr === 'stepper:numPolePairs') return 50;
        if (attr === 'stepper:dampingCoeff') return 0.01;
        if (attr === 'stepper:maxSpeedRad') return 600;
        if (attr === 'cable:linkable') return true;
      }
      if (prim?.path === '/World/SlideprinterScene/AttachA') {
        if (attr === 'ecs:tags') return ['Attachment'];
        if (attr === 'xformOp:translate') return [0.1, 0.2, 0.003];
        if (attr === 'radius') return 0.01;
        if (attr === 'physics:mass') return 0.027;
        if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
      }
      if (prim?.path === '/World/SlideprinterScene/Extruder') {
        if (attr === 'ecs:tags') return ['Extruder'];
        if (attr === 'xformOp:translate') return [0.0, 0.0, 0.0];
      }
      if (prim?.path === '/World/SlideprinterScene/Extruder/Tip') {
        if (attr === 'xformOp:translate') return [0.02, -0.02, -0.001];
      }
      if (prim?.path === '/World/SlideprinterScene/Extruder/ColdEnd') {
        if (attr === 'xformOp:translate') return [0.02, -0.02, 0.049];
      }
      return null;
    });
    usdStage.getChild.mockImplementation((prim, name) => {
      if (prim?.path === '/World/SlideprinterScene/Extruder' && (name === 'Tip' || name === 'ColdEnd')) {
        return {
          path: `/World/SlideprinterScene/Extruder/${name}`,
          name,
          type: 'definition',
          defType: 'Xform',
        };
      }
      return null;
    });
    usdStage.getRelationship.mockImplementation((prim, rel) => {
      if (prim?.path === '/World/SlideprinterScene/Extruder' && rel === 'machine:centerSources') {
        return ['/World/SlideprinterScene/AttachA'];
      }
      return [];
    });

    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    setupScene(world, stage, createCanvas());

    const extruderEntity = world.query([ExtruderComponent])[0];
    const extruder = world.getComponent(extruderEntity, ExtruderComponent);

    expect(extruder.centerOffsets.default.x).toBeCloseTo(0.0, 6);
    expect(extruder.centerOffsets.default.y).toBeCloseTo(0.0, 6);
    expect(extruder.centerOffsets.default.z).toBeCloseTo(0.0, 6);
    expect(extruder.tipOffsets.default.x).toBeCloseTo(0.02, 6);
    expect(extruder.tipOffsets.default.y).toBeCloseTo(-0.02, 6);
    expect(extruder.tipOffsets.default.z).toBeCloseTo(-0.001, 6);
    expect(extruder.coldEndOffsets.default.z).toBeCloseTo(0.049, 6);
    expect(extruder.centerPos.x).toBeCloseTo(0.1, 6);
    expect(extruder.centerPos.y).toBeCloseTo(0.2, 6);
    expect(extruder.centerPos.z).toBeCloseTo(0.003, 6);
    expect(extruder.tipPos.x).toBeCloseTo(0.12, 6);
    expect(extruder.tipPos.y).toBeCloseTo(0.18, 6);
    expect(extruder.tipPos.z).toBeCloseTo(0.002, 6);
    expect(extruder.coldEndPos.z).toBeCloseTo(0.052, 6);
  });

  test('loads Wheel prims as passive rotating cylinders without stepper controls', () => {
    usdStage.getChildren.mockImplementation((prim) => {
      if (prim?.path === '/World/SlideprinterScene') {
        return [{
          path: '/World/SlideprinterScene/WheelA',
          name: 'WheelA',
          type: 'definition',
          defType: 'Circle',
        }];
      }
      return [];
    });
    usdStage.getAttribute.mockImplementation((prim, attr) => {
      if (prim?.path === '/World/PhysicsScene') {
        if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
        if (attr === 'physics:gravityMagnitude') return 9.82;
      }
      if (prim?.path === '/World/SlideprinterScene/WheelA') {
        if (attr === 'ecs:tags') return ['Wheel'];
        if (attr === 'xformOp:translate') return [0.2, 0.3, 0.4];
        if (attr === 'xformOp:orient') return [1.0, 0.0, 0.0, 0.0];
        if (attr === 'radius') return 0.05;
        if (attr === 'physics:mass') return 1.0;
        if (attr === 'physics:inertiaTensor') return [[0.01, 0, 0], [0, 0.02, 0], [0, 0, 0.03]];
        if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
        if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
        if (attr === 'spool:axisLocal') return [1.0, 0.0, 0.0];
        if (attr === 'cable:linkable') return true;
      }
      return null;
    });
    usdStage.getRelationship.mockImplementation(() => []);

    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    setupScene(world, stage, createCanvas());

    const wheelEntities = world.query([
      SpoolStateComponent,
      RenderableComponent,
      EncoderComponent,
      OrientationComponent,
      AngularVelocityComponent,
    ]);
    expect(wheelEntities).toHaveLength(1);

    const wheelEntity = wheelEntities[0];
    const spoolState = world.getComponent(wheelEntity, SpoolStateComponent);
    const renderable = world.getComponent(wheelEntity, RenderableComponent);
    const cableLink = world.getComponent(wheelEntity, CableLinkComponent);

    expect(renderable.shape).toBe('cylinder');
    expect(spoolState.axis).toBeNull();
    expect(spoolState.axisLocal.x).toBeCloseTo(1.0, 6);
    expect(spoolState.axisLocal.y).toBeCloseTo(0.0, 6);
    expect(spoolState.axisLocal.z).toBeCloseTo(0.0, 6);
    expect(cableLink).toBeTruthy();
    expect(cableLink.cablePlaneNormalLocal.x).toBeCloseTo(1.0, 6);
    expect(world.getComponent(wheelEntity, StepperMotorComponent)).toBeUndefined();
    expect(world.getComponent(wheelEntity, SpoolTagComponent)).toBeUndefined();
  });

  test('accepts xformOp:rotateYXZ using USD angle packing', () => {
    usdStage.getChildren.mockImplementation((prim) => {
      if (prim?.path === '/World/SlideprinterScene') {
        return [{
          path: '/World/SlideprinterScene/WheelA',
          name: 'WheelA',
          type: 'definition',
          defType: 'Circle',
        }];
      }
      return [];
    });
    usdStage.getAttribute.mockImplementation((prim, attr) => {
      if (prim?.path === '/World/PhysicsScene') {
        if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
        if (attr === 'physics:gravityMagnitude') return 9.82;
      }
      if (prim?.path === '/World/SlideprinterScene/WheelA') {
        if (attr === 'ecs:tags') return ['Wheel'];
        if (attr === 'xformOp:translate') return [0.2, 0.3, 0.4];
        if (attr === 'xformOp:rotateYXZ') return [0.0, 90.0, 0.0];
        if (attr === 'xformOpOrder') return ['xformOp:rotateYXZ'];
        if (attr === 'radius') return 0.05;
        if (attr === 'physics:mass') return 1.0;
        if (attr === 'physics:inertiaTensor') return [[0.01, 0, 0], [0, 0.02, 0], [0, 0, 0.03]];
        if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
        if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
      }
      return null;
    });
    usdStage.getRelationship.mockImplementation(() => []);

    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    setupScene(world, stage, createCanvas());

    const wheelEntity = world.query([OrientationComponent])[0];
    const orientation = world.getComponent(wheelEntity, OrientationComponent).quaternion;
    const rotatedForward = orientation.transformVector(new Vector3(0.0, 0.0, 1.0));
    const expected = new Quaternion()
      .setFromAxisAngle(new Vector3(0.0, 1.0, 0.0), Math.PI / 2.0)
      .transformVector(new Vector3(0.0, 0.0, 1.0));

    expect(rotatedForward.x).toBeCloseTo(expected.x, 6);
    expect(rotatedForward.y).toBeCloseTo(expected.y, 6);
    expect(rotatedForward.z).toBeCloseTo(expected.z, 6);
  });

  test('does not treat a 3-value xformOp:orient as Euler shorthand', () => {
    usdStage.getChildren.mockImplementation((prim) => {
      if (prim?.path === '/World/SlideprinterScene') {
        return [{
          path: '/World/SlideprinterScene/WheelA',
          name: 'WheelA',
          type: 'definition',
          defType: 'Circle',
        }];
      }
      return [];
    });
    usdStage.getAttribute.mockImplementation((prim, attr) => {
      if (prim?.path === '/World/PhysicsScene') {
        if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
        if (attr === 'physics:gravityMagnitude') return 9.82;
      }
      if (prim?.path === '/World/SlideprinterScene/WheelA') {
        if (attr === 'ecs:tags') return ['Wheel'];
        if (attr === 'xformOp:translate') return [0.2, 0.3, 0.4];
        if (attr === 'xformOp:orient') return [0.0, 90.0, 0.0];
        if (attr === 'xformOpOrder') return ['xformOp:orient'];
        if (attr === 'radius') return 0.05;
        if (attr === 'physics:mass') return 1.0;
        if (attr === 'physics:inertiaTensor') return [[0.01, 0, 0], [0, 0.02, 0], [0, 0, 0.03]];
        if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
        if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
      }
      return null;
    });
    usdStage.getRelationship.mockImplementation(() => []);

    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    setupScene(world, stage, createCanvas());

    const wheelEntity = world.query([OrientationComponent])[0];
    const orientation = world.getComponent(wheelEntity, OrientationComponent).quaternion;
    const rotatedForward = orientation.transformVector(new Vector3(0.0, 0.0, 1.0));

    expect(rotatedForward.x).toBeCloseTo(0.0, 6);
    expect(rotatedForward.y).toBeCloseTo(0.0, 6);
    expect(rotatedForward.z).toBeCloseTo(1.0, 6);
  });

  test('treats Eyelet like Pinhole and assigns a fallback render radius', () => {
    usdStage.getChildren.mockImplementation((prim) => {
      if (prim?.path === '/World/SlideprinterScene') {
        return [{
          path: '/World/SlideprinterScene/EyeletA',
          name: 'EyeletA',
          type: 'definition',
          defType: 'Circle',
        }];
      }
      return [];
    });
    usdStage.getAttribute.mockImplementation((prim, attr) => {
      if (prim?.path === '/World/PhysicsScene') {
        if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
        if (attr === 'physics:gravityMagnitude') return 9.82;
      }
      if (prim?.path === '/World/SlideprinterScene/EyeletA') {
        if (attr === 'ecs:tags') return ['Eyelet'];
        if (attr === 'xformOp:translate') return [0.1, 0.2, 0.3];
        if (attr === 'physics:mass') return 0.0002;
        if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
        if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
      }
      return null;
    });
    usdStage.getRelationship.mockImplementation(() => []);

    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    setupScene(world, stage, createCanvas());

    const eyeletEntities = world.query([RenderableComponent, RadiusComponent, PositionComponent]);
    expect(eyeletEntities).toHaveLength(1);

    const eyeletEntity = eyeletEntities[0];
    const renderable = world.getComponent(eyeletEntity, RenderableComponent);
    const radius = world.getComponent(eyeletEntity, RadiusComponent);
    const pos = world.getComponent(eyeletEntity, PositionComponent).pos;

    expect(renderable.shape).toBe('circle');
    expect(radius.radius).toBeCloseTo(0.002, 9);
    expect(pos.x).toBeCloseTo(0.1, 9);
    expect(pos.y).toBeCloseTo(0.2, 9);
    expect(pos.z).toBeCloseTo(0.3, 9);
  });
});
