// Minimal mechanism repro for Finding 1: one Wheel prim with
// xformOpOrder = ["xformOp:rotateXYZ", "xformOp:translate"].
//
// USD composes this as R * T, so rotateXYZ (0,90,0) maps translate (1,0,0)
// to world (0,0,-1).  The loader reads the translate directly and places the
// wheel at (1,0,0): a 1.414 m error.
import { World } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/ecs.js';
import { PositionComponent, SceneEntityInfoComponent } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/ecs.js';
import { OpenText } from '/home/torbjorn/repos/hp-sim5/src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '/home/torbjorn/repos/hp-sim5/src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '/home/torbjorn/repos/hp-sim5/hp-sim-3d/app/scene/machineScenePipeline.js';

const source = `#usda 1.0
(
    metersPerUnit = 1
    timeCodesPerSecond = 500
)
def Xform "World"
{
    def PhysicsScene "PhysicsScene"
    {
        vector3d physics:gravityDirection = (0, 0, -1)
        double physics:gravityMagnitude = 9.81
    }
    def Xform "Scene"
    {
        def Circle "Wheel"
        {
            token[] ecs:tags = ["Wheel"]
            double3 xformOp:translate = (1, 0, 0)
            double3 xformOp:rotateXYZ = (0, 90, 0)
            uniform token[] xformOpOrder = ["xformOp:rotateXYZ", "xformOp:translate"]
            double radius = 0.1
            double height = 0.04
            double physics:mass = 0.4
            double3 physics:velocity = (0, 0, 0)
            double3 physics:angularVelocity = (0, 0, 0)
            matrix3d physics:inertiaTensor = ((0.02, 0, 0), (0, 0.03, 0), (0, 0, 0.04))
        }
    }
}
`;

const stage = OpenText(bakeCableSceneUsdaSource(source).source);
const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(stage), '/World/Scene', {}));
if (!checked.valid) throw new Error(checked.warnings.join('\n'));
const world = new World();
applyEntityPlan(world, buildEntityPlan(checked, {}));

let wheel;
for (const id of world.query([SceneEntityInfoComponent])) {
  if (world.getComponent(id, SceneEntityInfoComponent).name === 'Wheel') wheel = id;
}
if (wheel === undefined) throw new Error('probe failed to build the Wheel entity');
const pos = world.getComponent(wheel, PositionComponent).pos;
console.log(`loader position: (${pos.x}, ${pos.y}, ${pos.z})`);
console.log('USD position:    (0, 0, -1)');
const error = Math.hypot(pos.x - 0, pos.y - 0, pos.z + 1);
console.log(`error: ${(1000 * error).toFixed(1)} mm`);
if (error > 0.001) {
  console.log('FAIL: xformOp:translate is not composed through the declared rotateXYZ');
  process.exit(1);
}
console.log('PASS');
