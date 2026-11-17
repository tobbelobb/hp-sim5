// Two USD scenes that describe the same physical machine: one in metres
// (fixture, metersPerUnit = 1) and one authored in millimetres
// (metersPerUnit = 0.001, every linear value x1000).  A USD-correct reader
// returns the same world geometry for both.  The simulator never reads
// metersPerUnit, so it loads the millimetre scene 1000x too large.
import { readFileSync } from 'node:fs';
import { World } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/ecs.js';
import { PositionComponent } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/ecs.js';
import { SceneEntityInfoComponent } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints_3d/ecs.js';
import { OpenText } from '/home/torbjorn/repos/hp-sim5/src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '/home/torbjorn/repos/hp-sim5/src/js/usd/cable_scene_baker.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '/home/torbjorn/repos/hp-sim5/hp-sim-3d/app/scene/machineScenePipeline.js';

const ROOT = '/home/torbjorn/repos/hp-sim5';
const fixture = readFileSync(`${ROOT}/tests/fixtures/engine_parity/usd_scene_minimal.usda`, 'utf8');

function scaleLinear(text, factor) {
  return text
    .replace(/(xformOp:translate = \()([^)]*)(\))/g, (_, a, body, c) =>
      a + body.split(',').map((v) => String(Number(v) * factor)).join(', ') + c)
    .replace(/(double radius = )([-0-9.eE+]+)/g, (_, a, v) => a + String(Number(v) * factor))
    .replace(/(physics:(?:min|max)Distance = )([-0-9.eE+]+)/g, (_, a, v) => a + String(Number(v) * factor));
}

const millimetre = fixture
  .replace('(\n    timeCodesPerSecond = 500\n)', '(\n    metersPerUnit = 0.001\n    timeCodesPerSecond = 500\n)')
  .replace(/#usda 1.0\n\(\n    timeCodesPerSecond = 500\n\)/, '#usda 1.0\n(\n    metersPerUnit = 0.001\n    timeCodesPerSecond = 500\n)');
const mmScene = scaleLinear(millimetre, 1000);
if (!mmScene.includes('metersPerUnit = 0.001')) {
  throw new Error('probe failed to insert metersPerUnit');
}

function load(source) {
  const stage = OpenText(bakeCableSceneUsdaSource(source).source);
  const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(stage), '/World/Scene', {}));
  if (!checked.valid) throw new Error(checked.warnings.join('\n'));
  const world = new World();
  applyEntityPlan(world, buildEntityPlan(checked, {}));
  const byName = new Map();
  for (const id of world.query([SceneEntityInfoComponent])) {
    const info = world.getComponent(id, SceneEntityInfoComponent);
    const pos = world.getComponent(id, PositionComponent)?.pos;
    if (pos) byName.set(info.name, [pos.x, pos.y, pos.z]);
  }
  return byName;
}

const metres = load(fixture);
const millimetres = load(mmScene);
let failed = false;
for (const name of ['Anchor', 'Wheel', 'SpoolA', 'Corner']) {
  const a = metres.get(name);
  const b = millimetres.get(name);
  const ratio = b ? Math.hypot(...b) / Math.hypot(...a) : NaN;
  console.log(`${name}: metres=${a} millimetres=${b} ratio=${ratio.toFixed(1)}`);
  const errMm = 1000 * Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  if (errMm > 1.0) {
    console.log(`  BUG: same USD geometry differs by ${errMm.toFixed(1)} mm`);
    failed = true;
  }
}
if (failed) {
  console.log('FAIL: metersPerUnit is ignored; a valid millimetre scene loads 1000x too large');
  process.exit(1);
}
console.log('PASS');
