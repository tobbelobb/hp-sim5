// Rebuild the opt-in gate scene from the production HP4 geometry.
import fs from 'node:fs';
import { createHeadlessWorld } from '../hp-sim-3d/app/headlessWorld.js';
import { SceneEntityInfoComponent, PositionComponent } from '../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent } from '../src/js/cable_joints_3d/cable_joints_core.js';
import { computeLocalAttachment } from '../src/js/cable_joints_3d/rigid_bodies.js';
import { bakeCableSceneUsdaSource } from '../src/js/usd/cable_scene_baker.js';
import { OpenText, getAttribute } from '../src/js/usd/stage.js';
const source = fs.readFileSync(new URL('../public/usd_scenes/hp4_rigid_body.usda', import.meta.url), 'utf8');
const stage = OpenText(bakeCableSceneUsdaSource(source).source);
const { world } = createHeadlessWorld(source, { cableSolverBackend: null });
const entities = new Map(world.query([SceneEntityInfoComponent]).map(id => [world.getComponent(id, SceneEntityInfoComponent).name, id]));
// Original HP4 encoder replay shows the two reeving bearings counter-rotating.
// The bottom/top magnitude is a provisional visual approximation; the original
// ratio varied by branch and trial, so this is not a calibrated wheel model.
const wheelRotationScale = { top: 1.0, bottom: -0.338 };
let scene = source.replace('A USD scene for Hangprinter v4', 'Hangprinter v4 with stateful two-port pinhole gates (experimental)');
for (const branch of ['AL','AR','BL','BR','CL','CR']) {
  const root = '/World/HangprinterScene/';
  const path = stage.GetPrimAtPath(root + 'CablePath' + branch);
  const stored = getAttribute(path, 'cablePath:stored');
  const middle = ['pinhole_to_wheel', 'wheel_to_wheel', 'wheel_to_pinhole'];
  const hidden = middle.reduce((sum, name) => sum + getAttribute(stage.GetPrimAtPath(root + `Joint${branch}_${name}`), 'restLength'), 0) + stored[3] + stored[4];
  scene = scene.replace(new RegExp(`        def CableJoint "Joint${branch}_pinhole_to_wheel"[\\s\\S]*?(?=        def CableJoint "Joint${branch}_pinhole_to_anchor")`), '');
  const entry = entities.get(`Pinhole${branch}_top`);
  const points = [world.getComponent(entry, PositionComponent).pos];
  for (const name of middle) {
    const joint = world.getComponent(entities.get(`Joint${branch}_${name}`), CableJointComponent);
    points.push(joint.attachmentPointA_world, joint.attachmentPointB_world);
  }
  points.push(world.getComponent(entities.get(`Pinhole${branch}_bottom`), PositionComponent).pos);
  const visual = points.map(p => computeLocalAttachment(world, entry, p)).map(p => `(${p.x}, ${p.y}, ${p.z})`).join(', ');
  const wheelPaths = ['top', 'bottom'].map(side => root + `Wheel${branch}_${side}`);
  const wheels = wheelPaths.map(path => stage.GetPrimAtPath(path));
  const radius = getAttribute(wheels[0], 'radius');
  const effectiveInertia = wheels.reduce((sum, wheel) => {
    const inertia = getAttribute(wheel, 'physics:inertiaTensor');
    const r = getAttribute(wheel, 'radius');
    return sum + inertia[2][2] / (r * r);
  }, 0.0);
  // Provisional dry-bearing losses. These are synthetic until measurements are
  // available; they are equivalent cable-flow values, not fit HP4 results.
  const bearingViscousTorque = 2e-5;
  const bearingCoulombTorque = 2e-5;
  const damping = wheels.reduce((sum, wheel) => {
    const r = getAttribute(wheel, 'radius');
    return sum + bearingViscousTorque / (r * r);
  }, 0.0);
  const coulombTorque = bearingCoulombTorque * wheels.length;
  scene = scene.replace(new RegExp(`(        def Circle "Pinhole${branch}_top"[\\s\\S]*?)(\\n        })`), (_, block, end) => block + `\n            custom rel pinholeGate:exit = <${root}Pinhole${branch}_bottom>\n            custom rel pinholeGate:wheels = [${wheelPaths.map(path => `<${path}>`).join(', ')}]\n            custom point3d[] pinholeGate:visualRoute = [${visual}]\n            custom bool pinholeGate:visible = 1\n            custom double pinholeGate:effectiveInertia = ${effectiveInertia}\n            custom double pinholeGate:damping = ${damping}\n            custom double pinholeGate:coulombTorque = ${coulombTorque}\n            custom double pinholeGate:radius = ${radius}` + end);
  for (const side of ['top', 'bottom']) {
    scene = scene.replace(new RegExp(`(        def Circle "Wheel${branch}_${side}"[\\s\\S]*?token\\[\\] ecs:tags = )\\["Wheel"\\]([\\s\\S]*?)(\\n        })`), `$1["Pinhole", "GateVisual"]$2\n            custom double pinholeGate:rotationScale = ${wheelRotationScale[side]}$3`);
  }
  scene = scene.replace(new RegExp(`        def Xform "CablePath${branch}"[\\s\\S]*?\\n        }`), block => block
    .replace(/custom rel cablePath:joints = .*/, `custom rel cablePath:joints = [<${root}Joint${branch}_0>, <${root}Joint${branch}_anchor_to_pinhole>, <${root}Joint${branch}_pinhole_to_anchor>]`)
    .replace(/custom token\[\] cablePath:linkTypes = .*/, 'custom token[] cablePath:linkTypes = ["hybrid", "pinhole", "pinhole_gate", "attachment"]')
    .replace(/custom bool\[\] cablePath:clockwise = .*/, 'custom bool[] cablePath:clockwise = [1, 1, 1, 1]')
    .replace(/custom double\[\] cablePath:stored = .*/, `custom double[] cablePath:stored = [2.0, 0, ${hidden}, 0]`)
    .replace(/custom token\[\] cablePath:storedMode = .*/, 'custom token[] cablePath:storedMode = ["manual", "manual", "manual", "manual"]')
    // Preserve six identical original Kelvin-Voigt spans in series.
    .replace('stiffness = 20000.0', 'stiffness = 3333.3333333333335')
    .replace('damping = 5.0', 'damping = 0.8333333333333334'));
}
fs.writeFileSync(new URL('../public/usd_scenes/hp4_pinhole_gate.usda', import.meta.url), scene);
