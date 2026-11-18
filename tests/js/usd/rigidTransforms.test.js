import { CableJointComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { PositionComponent, SceneEntityInfoComponent, World } from '../../../src/js/cable_joints_3d/ecs.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';
import { applyEntityPlan, buildEntityPlan, readMachineSceneSpec, validateMachineSceneSpec } from '../../../hp-sim-3d/app/scene/machineScenePipeline.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { OpenText } from '../../../src/js/usd/stage.js';

const source = `#usda 1.0
(timeCodesPerSecond = 500)
def Xform "World" {
    double3 xformOp:translate = (10, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
    def PhysicsScene "PhysicsScene" {
        vector3d physics:gravityDirection = (0, 0, -1)
        double physics:gravityMagnitude = 9.81
    }
    def Xform "Scene" {
        double xformOp:rotateZ = 90
        uniform token[] xformOpOrder = ["xformOp:rotateZ"]

        def Circle "Wheel" {
            token[] ecs:tags = ["Wheel"]
            double3 xformOp:translate:offset = (1, 0, 0)
            quatd xformOp:orient = (0.7071067811865476, 0, 0.7071067811865476, 0)
            uniform token[] xformOpOrder = ["xformOp:orient", "xformOp:translate:offset"]
            double radius = 0.1
            double height = 0.04
            double physics:mass = 0.4
            double3 physics:angularVelocity = (0, 0, 0)
            matrix3d physics:inertiaTensor = ((0.02, 0, 0), (0, 0.03, 0), (0, 0, 0.04))
        }
        def Xform "Anchor" {
            token[] ecs:tags = ["Anchor"]
            double3 xformOp:translate = (10, 3, -1)
            double3 xformOp:scale = (2, 2, 2)
            double3 xformOp:rotateXYZ = (0, 0, 90)
            uniform token[] xformOpOrder = ["xformOp:scale", "!resetXformStack!", "xformOp:translate", "xformOp:orient"]
        }
        def Xform "Pivot" {
            token[] ecs:tags = ["Anchor"]
            double3 xformOp:translate:pivot = (1, 0, 0)
            double3 xformOp:rotateXYZ = (0, 0, 90)
            uniform token[] xformOpOrder = ["xformOp:translate:pivot", "xformOp:rotateXYZ", "!invert!xformOp:translate:pivot"]
        }
        def Xform "EmptyOrder" {
            token[] ecs:tags = ["Anchor"]
            double3 xformOp:translate = (1, 2, 3)
            uniform token[] xformOpOrder = []
        }
        def Xform "AbsentOrder" {
            token[] ecs:tags = ["Anchor"]
            double3 xformOp:translate = (1, 2, 3)
        }
        def Xform "Extruder" {
            token[] ecs:tags = ["Extruder"]
            double3 xformOp:translate = (1, 0, 0)
            double xformOp:rotateZ = 90
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]
            def Xform "Tip" {
                double3 xformOp:translate = (0, 1, 0)
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }
            def Xform "ColdEnd" {
                double3 xformOp:translate = (10, 1, 3)
                uniform token[] xformOpOrder = ["!resetXformStack!", "xformOp:translate"]
            }
        }
        def CableJoint "Joint" {
            rel physics:body0 = </World/Scene/Wheel>
            rel physics:body1 = </World/Scene/Anchor>
            point3d localPos0 = (0, 0, 1)
            point3d localPos1 = (1, 0, 0)
        }
        def Xform "Path" (apiSchemas = ["CablePathAPI"]) {
            rel cablePath:joints = [</World/Scene/Joint>]
            token[] cablePath:linkTypes = ["attachment", "attachment"]
            bool[] cablePath:clockwise = [0, 0]
        }
    }
}
`;

function buildPlan(usda) {
  const spec = readMachineSceneSpec(OpenText(usda), '/World/Scene');
  return buildEntityPlan(validateMachineSceneSpec(spec));
}

function expectPoint(point, x, y, z) {
  expect(point.x).toBeCloseTo(x, 12);
  expect(point.y).toBeCloseTo(y, 12);
  expect(point.z).toBeCloseTo(z, 12);
}

test('scene loading and baking agree on ordered rigid ops, parent frames and omitted ops', () => {
  const baked = bakeCableSceneUsdaSource(source);
  const world = new World();
  applyEntityPlan(world, buildPlan(baked.source));
  const entities = Object.fromEntries(world.query([SceneEntityInfoComponent]).map(id => [
    world.getComponent(id, SceneEntityInfoComponent).name, id,
  ]));
  const position = name => world.getComponent(entities[name], PositionComponent).pos;

  expectPoint(position('Wheel'), 10, 0, -1);
  expectPoint(position('Anchor'), 10, 3, -1);
  expectPoint(position('Pivot'), 11, 1, 0);
  expectPoint(position('EmptyOrder'), 10, 0, 0);
  expectPoint(position('AbsentOrder'), 10, 0, 0);
  const joint = world.getComponent(entities.Joint, CableJointComponent);
  expectPoint(joint.attachmentPointA_world, 10, 1, -1);
  expectPoint(joint.attachmentPointB_world, 11, 3, -1);
  expect(joint.restLength).toBeCloseTo(Math.sqrt(5), 12);

  const extruder = world.getComponent(world.query([ExtruderComponent])[0], ExtruderComponent);
  expectPoint(extruder.centerOffsets.default, 0, 1, 0);
  expectPoint(extruder.tipOffsets.default, 0, -1, 0);
  expectPoint(extruder.coldEndOffsets.default, 0, 0, 3);
});

test('an active non-rigid transform identifies the prim and unsupported op', () => {
  const unsupported = source.replace('"xformOp:scale", "!resetXformStack!"', '"!resetXformStack!", "xformOp:scale"');
  const error = '/World/Scene/Anchor: unsupported USD transform op xformOp:scale.';
  expect(() => bakeCableSceneUsdaSource(unsupported)).toThrow(error);
  expect(() => buildPlan(unsupported)).toThrow(error);
});
