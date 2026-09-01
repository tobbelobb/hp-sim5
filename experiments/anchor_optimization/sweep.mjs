import { spawn } from 'node:child_process';
import { mkdir, readFile, writeFile, chmod } from 'node:fs/promises';
import path from 'node:path';
import { chromium } from 'playwright';

const ROOT = process.cwd();
const RESULT_DIR = path.join(ROOT, 'experiments/anchor_optimization/results');
const WORK_DIR = path.join(RESULT_DIR, 'work');
const VSD_DIR = path.join(ROOT, 'RRF/run/vsd');
const RRF_BIN = path.join(ROOT, 'RRF/build/rrf_simulator');
const BASE_URL = 'http://127.0.0.1:5173/hp-sim5';
const ANCHOR_AXES = ['A', 'B', 'C', 'D', 'I', 'J', 'L', 'O'];
const VISIBLE_AXES = ['X', 'Y', 'Z', 'U', 'V', 'W', 'A', 'B'];
const fmt = (value) => Number(value).toFixed(9);
const vec = (values) => `(${values.map(fmt).join(', ')})`;
const list = (value, count) => Array.from({ length: count }, () => value).join(':');

function parseArgs() {
  const parsed = {};
  for (const arg of process.argv.slice(2)) {
    const match = /^--([^=]+)=(.*)$/.exec(arg);
    if (match) parsed[match[1]] = match[2];
  }
  return {
    mode: parsed.mode === '3d' ? '3d' : '2d',
    count: Math.max(3, Math.min(8, Number(parsed.count) || 3)),
    smoke: parsed.smoke === '1',
    pattern: parsed.pattern === 'bigger' ? 'bigger' : 'small',
  };
}

function normalize(values) {
  const length = Math.hypot(...values);
  return values.map((value) => value / length);
}

function scale(values, amount) {
  return values.map((value) => value * amount);
}

function subtract(a, b) {
  return a.map((value, index) => value - b[index]);
}

function make2dCandidate(count, radius, phaseFraction) {
  const phase = phaseFraction * (2 * Math.PI / count);
  const anchors = [];
  for (let index = 0; index < count; index += 1) {
    const angle = phase + index * 2 * Math.PI / count;
    anchors.push([radius * Math.cos(angle), radius * Math.sin(angle), 0]);
  }
  return {
    id: `2d_n${count}_r${radius.toFixed(2).replace('.', 'p')}_p${phaseFraction.toFixed(3).replace('.', 'p')}`,
    mode: '2d',
    family: 'regular',
    count,
    radius,
    phaseFraction,
    anchorMode: 2,
    ignoreGravity: true,
    effectorMass: 0.081,
    anchors,
  };
}

function make3dCandidate(count, family, radius, elevation, phaseFraction, lowerOffsetFraction = 0.5) {
  const anchors = [];
  const phase = phaseFraction * (2 * Math.PI / Math.max(1, count));
  if (family === 'all_high') {
    const z = radius * elevation;
    const xyRadius = Math.sqrt(Math.max(0, radius * radius - z * z));
    for (let index = 0; index < count; index += 1) {
      const angle = phase + index * 2 * Math.PI / count;
      anchors.push([xyRadius * Math.cos(angle), xyRadius * Math.sin(angle), z]);
    }
  } else if (family === 'one_high') {
    const lowCount = count - 1;
    const lowZ = -radius * elevation;
    const xyRadius = Math.sqrt(Math.max(0, radius * radius - lowZ * lowZ));
    for (let index = 0; index < lowCount; index += 1) {
      const angle = phase + index * 2 * Math.PI / lowCount;
      anchors.push([xyRadius * Math.cos(angle), xyRadius * Math.sin(angle), lowZ]);
    }
    anchors.push([0, 0, radius]);
  } else {
    const half = count / 2;
    const z = radius * elevation;
    const xyRadius = Math.sqrt(Math.max(0, radius * radius - z * z));
    for (let index = 0; index < half; index += 1) {
      const angle = phase + index * 2 * Math.PI / half;
      anchors.push([xyRadius * Math.cos(angle), xyRadius * Math.sin(angle), z]);
    }
    const lowerOffset = lowerOffsetFraction * 2 * Math.PI / half;
    for (let index = 0; index < half; index += 1) {
      const angle = phase + lowerOffset + index * 2 * Math.PI / half;
      anchors.push([xyRadius * Math.cos(angle), xyRadius * Math.sin(angle), -z]);
    }
  }
  return {
    id: `3d_${family}_n${count}_r${radius.toFixed(2).replace('.', 'p')}_e${elevation.toFixed(2).replace('.', 'p')}_p${phaseFraction.toFixed(3).replace('.', 'p')}`,
    mode: '3d',
    family,
    count,
    radius,
    elevation,
    phaseFraction,
    lowerOffsetFraction,
    anchorMode: family === 'all_high' ? 2 : (family === 'one_high' ? 1 : 3),
    ignoreGravity: false,
    effectorMass: 0.108,
    anchors,
  };
}

function makeCandidates({ mode, count, smoke }) {
  if (mode === '2d') {
    if (smoke) return [make2dCandidate(count, 1.9, 0)];
    const candidates = [];
    for (const radius of [1.0, 1.25, 1.5, 1.75, 2.0]) {
      for (const phase of [0, 0.125, 0.25, 0.375]) {
        candidates.push(make2dCandidate(count, radius, phase));
      }
    }
    return candidates;
  }

  if (smoke) return [make3dCandidate(count, 'all_high', 1.9, 0.55, 0)];
  const candidates = [];
  for (const family of ['all_high', 'one_high']) {
    for (const radius of [1.5, 1.75, 2.0]) {
      for (const elevation of [0.35, 0.55, 0.75]) {
        for (const phase of [0, 0.25]) {
          candidates.push(make3dCandidate(count, family, radius, elevation, phase));
        }
      }
    }
  }
  if (count % 2 === 0) {
    for (const radius of [1.5, 1.75, 2.0]) {
      for (const elevation of [0.35, 0.55, 0.75]) {
        for (const offset of [0, 0.5]) {
          candidates.push(make3dCandidate(count, 'half', radius, elevation, 0, offset));
        }
      }
    }
  }
  return candidates;
}

function makeScene(candidate) {
  const is3d = candidate.mode === '3d';
  const sceneName = is3d ? 'HangprinterScene' : 'SlideprinterScene';
  const attachmentRadius = 0.2;
  const spoolOffset = 0.12;
  const memberMass = candidate.effectorMass / candidate.count;
  const members = [];
  const bodies = [];
  const joints = [];
  const paths = [];
  const centerSources = [];
  const effectiveAnchors = [];

  candidate.anchors.forEach((anchor, index) => {
    const axis = ANCHOR_AXES[index];
    const direction = normalize(anchor);
    const attachment = scale(direction, attachmentRadius);
    const spool = anchor.map((value, coordinate) => value + direction[coordinate] * spoolOffset);
    effectiveAnchors.push(subtract(anchor, attachment));
    members.push(`</World/${sceneName}/Attach${axis}>`);
    centerSources.push(`</World/${sceneName}/Attach${axis}>`);

    bodies.push(`
        def Circle "Spool${axis}" (
            apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "MaterialBindingAPI"]
        )
        {
            token[] ecs:tags = ["Spool", "Stepper"]
            rel material:binding = </World/PhysicsScene/SpoolMaterial>
            custom bool cable:linkable = 1
            double radius = 0.03
            quatf xformOp:orient = (1, 0, 0, 0)
            double3 xformOp:translate = ${vec(spool)}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
            float physics:mass = -1.0
            float3 physics:angularVelocity = (0, 0, 0)
            matrix3d physics:inertiaTensor = ((5e-5, 0, 0), (0, 5e-5, 0), (0, 0, 5e-5))
            custom double stepper:holdingTorque = 0.5
            custom int stepper:numPolePairs = 50
            custom double stepper:dampingCoeff = 0.011
            vector3f physics:velocity = (0, 0, 0)
        }

        def Xform "Anchor${axis}"
        {
            token[] ecs:tags = ["Anchor"]
            double3 xformOp:translate = ${vec(anchor)}
            uniform token[] xformOpOrder = ["xformOp:translate"]
            rel material:binding = </World/PhysicsScene/AnchorMaterial>
        }

        def Circle "Attach${axis}" (
            apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "MaterialBindingAPI"]
        )
        {
            token[] ecs:tags = ["Attachment"]
            double radius = 0.01
            double3 xformOp:translate = ${vec(attachment)}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
            float physics:mass = ${memberMass.toFixed(9)}
            vector3f physics:velocity = (0, 0, 0)
            rel material:binding = </World/PhysicsScene/PinholeMaterial>
        }`);

    joints.push(`
        def CableJoint "Joint${axis}_0"
        {
            custom rel physics:body0 = </World/${sceneName}/Spool${axis}>
            custom rel physics:body1 = </World/${sceneName}/Anchor${axis}>
        }
        def CableJoint "Joint${axis}_1"
        {
            custom rel physics:body0 = </World/${sceneName}/Anchor${axis}>
            custom rel physics:body1 = </World/${sceneName}/Attach${axis}>
        }`);

    paths.push(`
        def Xform "CablePath${axis}" (
            apiSchemas = ["CablePathAPI"]
        )
        {
            custom bool[] cablePath:clockwise = [1, 1, 1]
            custom rel cablePath:joints = [</World/${sceneName}/Joint${axis}_0>, </World/${sceneName}/Joint${axis}_1>]
            custom token[] cablePath:linkTypes = ["hybrid", "pinhole", "attachment"]
            custom double[] cablePath:stored = [1.0, 0, 0]
            custom double cablePath:stiffness = 20000.0
            custom double cablePath:damping = 5.0
            custom double cablePath:halfWidth = 0.001
            custom token[] cablePath:storedMode = ["manual", "manual", "manual"]
            custom token cablePath:initPolicy = "deriveMissing"
        }`);
  });

  const rigidDefinition = is3d
    ? `def RigidBody "Effector" { rel rigidBody:members = [${members.join(', ')}] }`
    : `def RigidGroup "Effector" { rel rigidGroup:members = [${members.join(', ')}] }`;
  const rigidRelationship = is3d ? 'machine:rigidBody' : 'machine:rigidGroup';

  const source = `#usda 1.0
(
    doc = "Parametric anchor optimization candidate ${candidate.id}"
    defaultPrim = "${sceneName}"
    metersPerUnit = 1
    startTimeCode = 0
    timeCodesPerSecond = 500
    upAxis = "Y"
)

def Xform "World"
{
    def Xform "${sceneName}"
    {
${bodies.join('\n')}
        def Xform "Extruder"
        {
            token[] ecs:tags = ["Extruder"]
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
            rel machine:centerSources = [${centerSources.join(', ')}]
            rel ${rigidRelationship} = </World/${sceneName}/Effector>
            rel material:binding = </World/PhysicsScene/AnchorMaterial>
            def Xform "Tip"
            {
                double3 xformOp:translate = (0, 0, 0)
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }
            def Xform "ColdEnd"
            {
                double3 xformOp:translate = (0, 0, 0.1)
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }
        }

        ${rigidDefinition}
${joints.join('\n')}
${paths.join('\n')}
        custom color3f machine:tintColor = (0.1, 0.7, 0.9)
        custom color3f machine:extrusionColor = (0.1, 0.7, 0.9)
    }

    def PhysicsScene "PhysicsScene"
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.82
        def Material "SpoolMaterial" (
            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
        )
        {
            float physics:staticFriction = 0.2
            float physics:dynamicFriction = 0.2
            float physics:restitution = 0.5
        }
        def Material "PinholeMaterial" (
            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
        )
        {
            float physics:staticFriction = 0.005
            float physics:dynamicFriction = 0.005
            float physics:restitution = 0.025
        }
        def Material "AnchorMaterial" (
            prepend apiSchemas = ["PhysicsMaterialAPI"]
        )
        {
        }
    }
}
`;

  return { source, effectiveAnchors };
}

async function makeRrfConfig(candidate, effectiveAnchors, configPath) {
  let config = await readFile(path.join(VSD_DIR, 'sys/config_cubecorners.g'), 'utf8');
  const count = candidate.count;
  const m584 = VISIBLE_AXES.slice(0, count)
    .map((axis, index) => `${axis}${40 + index}.0`)
    .join(' ');
  const anchorLine = effectiveAnchors
    .map((anchor, index) => `${ANCHOR_AXES[index]}${anchor.map((value) => (value * 1000).toFixed(6)).join(':')}`)
    .join(' ');
  const minForce = candidate.mode === '2d' ? 3 : 3;
  const replacements = [
    [/^M584 .*$/m, `M584 ${m584} E48.0 P${count}`],
    [/^M669 N\d+.*$/m, `M669 N${count}`],
    [/^M666 A\d+.*$/m, `M666 A${candidate.anchorMode}`],
    [/^M669 A.*$/m, `M669 ${anchorLine}`],
    [/^M666 Q.*$/m, `M666 Q0.0 R${list('30.0', count)}`],
    [/^M666 U.*$/m, `M666 U${list('1', count)}`],
    [/^M666 O.*$/m, `M666 O${list('1', count)}`],
    [/^M666 L.*$/m, `M666 L${list('20', count)}`],
    [/^M666 H.*$/m, `M666 H${list('20', count)}`],
    [/^M666 W.*$/m, `M666 W${candidate.effectorMass.toFixed(6)}`],
    [/^M666 S.*$/m, 'M666 S20000.0'],
    [/^M666 I.*$/m, `M666 I${list(minForce.toFixed(1), count)}`],
    [/^M666 X.*$/m, `M666 X${list('170.0', count)}`],
    [/^M666 Y.*$/m, `M666 Y${list('116.19', count)}`],
    [/^M666 B.*$/m, `M666 B${candidate.ignoreGravity ? 1 : 0}`],
    [/^M666 P.*$/m, 'M666 P0'],
    [/^M666 J.*$/m, `M666 J${list('200', count)}`],
  ];
  for (const [pattern, value] of replacements) {
    if (!pattern.test(config)) throw new Error(`Config template pattern not found: ${pattern}`);
    config = config.replace(pattern, value);
  }
  await writeFile(configPath, config);
}

async function runProcess(command, processArgs, options = {}) {
  return new Promise((resolve, reject) => {
    const child = spawn(command, processArgs, { ...options, stdio: ['ignore', 'pipe', 'pipe'] });
    let stdout = '';
    let stderr = '';
    let timedOut = false;
    const timer = setTimeout(() => {
      timedOut = true;
      child.kill('SIGKILL');
    }, 120000);
    child.stdout.on('data', (chunk) => { stdout += chunk; });
    child.stderr.on('data', (chunk) => { stderr += chunk; });
    child.on('error', (error) => {
      clearTimeout(timer);
      reject(error);
    });
    child.on('close', (code) => {
      clearTimeout(timer);
      if (code === 0 && !timedOut) resolve({ stdout, stderr });
      else reject(new Error(`${command} ${timedOut ? 'timed out' : `exited ${code}`}\n${stdout}\n${stderr}`));
    });
  });
}

async function waitForServer(url, timeoutMs = 120000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    try {
      const response = await fetch(url);
      if (response.ok) return;
    } catch {}
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
  throw new Error('Vite server timeout');
}

async function measureCandidate(browser, candidate, usdaPath, csvPath) {
  const route = candidate.mode === '3d' ? 'hp-sim-3d' : 'hp-sim';
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const consoleErrors = [];
  page.on('console', (message) => {
    if (message.type() === 'error') consoleErrors.push(message.text());
  });
  page.on('pageerror', (error) => consoleErrors.push(error.message));
  const startedAt = Date.now();

  await page.goto(`${BASE_URL}/${route}/`, { waitUntil: 'networkidle', timeout: 120000 });
  await page.waitForFunction(() => {
    const button = document.querySelector('#printSquareBtn');
    return button && !button.disabled;
  }, null, { timeout: 120000 });

  await page.evaluate(async () => {
    window.__anchorOptimizationMetrics = [];
    window.__anchorOptimizationMonitors = [];
    const moduleUrl = new URL('./app/quality-monitor.js', window.location.href).href;
    const { QualityMonitor } = await import(moduleUrl);
    if (!QualityMonitor.prototype.__anchorOptimizationPatched) {
      const original = QualityMonitor.prototype.runFinalCheck;
      QualityMonitor.prototype.runFinalCheck = function patchedRunFinalCheck(...args) {
        const result = original.apply(this, args);
        window.__anchorOptimizationMetrics.push({
          machineId: this.machineId,
          machineLabel: this.machineLabel,
          metrics: this.getMetrics(),
          motorDiagnostics: this._getMotorDiagnostics(),
        });
        return result;
      };
      const originalSetMachineContext = QualityMonitor.prototype.setMachineContext;
      QualityMonitor.prototype.setMachineContext = function patchedSetMachineContext(...args) {
        if (!window.__anchorOptimizationMonitors.includes(this)) {
          window.__anchorOptimizationMonitors.push(this);
        }
        return originalSetMachineContext.apply(this, args);
      };
      QualityMonitor.prototype.__anchorOptimizationPatched = true;
    }
  });

  await page.click('#machinesRemoveAllBtn', { force: true });
  await page.waitForTimeout(100);
  await page.locator('#gcodeFile').setInputFiles(usdaPath);
  await page.waitForFunction(() => {
    const button = document.querySelector('#printSquareBtn');
    return button && !button.disabled;
  }, null, { timeout: 120000 });
  await page.locator('#gcodeFile').setInputFiles(csvPath);
  await page.waitForFunction(() => {
    const button = document.querySelector('#finishAsapBtn');
    return button && !button.disabled;
  }, null, { timeout: 120000 });
  for (let index = 0; index < 4; index += 1) {
    await page.click('#speedFasterBtn', { force: true });
  }
  let completionTimedOut = false;
  try {
    await page.waitForFunction(
      () => Array.isArray(window.__anchorOptimizationMetrics)
        && window.__anchorOptimizationMetrics.length > 0,
      null,
      { timeout: args.pattern === 'bigger' ? 360000 : 180000 }
    );
  } catch (error) {
    completionTimedOut = true;
    await page.evaluate(() => {
      for (const monitor of window.__anchorOptimizationMonitors || []) {
        monitor.runFinalCheck();
      }
    });
    await page.waitForFunction(
      () => Array.isArray(window.__anchorOptimizationMetrics)
        && window.__anchorOptimizationMetrics.length > 0,
      null,
      { timeout: 10000 }
    );
  }

  const records = await page.evaluate(() => window.__anchorOptimizationMetrics);
  const hud = await page.locator('#qualityHud').innerText().catch(() => '');
  await page.close();
  return {
    elapsedSeconds: (Date.now() - startedAt) / 1000,
    completionTimedOut,
    records,
    hud,
    consoleErrors,
  };
}

const args = parseArgs();
await mkdir(WORK_DIR, { recursive: true });
await mkdir(path.join(VSD_DIR, 'logs'), { recursive: true });
await chmod(RRF_BIN, 0o755).catch(() => {});
const candidates = makeCandidates(args);
const vite = spawn('npx', ['vite', '--host', '127.0.0.1', '--port', '5173'], {
  stdio: ['ignore', 'pipe', 'pipe'],
});
vite.stdout.on('data', (chunk) => process.stdout.write(chunk));
vite.stderr.on('data', (chunk) => process.stderr.write(chunk));

let browser;
const results = [];
try {
  await waitForServer(`${BASE_URL}/hp-sim/`);
  browser = await chromium.launch({
    headless: true,
    args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader'],
  });

  for (const candidate of candidates) {
    const { source, effectiveAnchors } = makeScene(candidate);
    const usdaPath = path.join(WORK_DIR, `${candidate.id}.usda`);
    const configName = `anchor_opt_${candidate.id}.g`;
    const configPath = path.join(VSD_DIR, 'sys', configName);
    const gcodeStem = args.pattern === 'bigger' ? 'draw_squares_bigger' : 'draw_squares';
    const logName = `${gcodeStem}_${candidate.id}.csv`;
    const csvPath = path.join(VSD_DIR, 'logs', logName);
    await writeFile(usdaPath, source);
    await makeRrfConfig(candidate, effectiveAnchors, configPath);

    const result = {
      candidate: {
        ...candidate,
        maxAnchorNorm: Math.max(...candidate.anchors.map((anchor) => Math.hypot(...anchor))),
        effectiveAnchors,
      },
      rrf: null,
      simulation: null,
      error: null,
    };
    try {
      console.log(`ANCHOR_OPT_PHASE=${candidate.id}:rrf:start`);
      const rrf = await runProcess(RRF_BIN, [
        '--vsd', VSD_DIR,
        '--gcode', `gcodes/${gcodeStem}.gcode`,
        '--can-log', `logs/${logName}`,
        '-c', `sys/${configName}`,
      ]);
      console.log(`ANCHOR_OPT_PHASE=${candidate.id}:rrf:done`);
      result.rrf = {
        stdoutTail: rrf.stdout.slice(-2000),
        stderrTail: rrf.stderr.slice(-2000),
      };
      console.log(`ANCHOR_OPT_PHASE=${candidate.id}:physics:start`);
      result.simulation = await measureCandidate(browser, candidate, usdaPath, csvPath);
      console.log(`ANCHOR_OPT_PHASE=${candidate.id}:physics:done`);
    } catch (error) {
      result.error = error?.stack || String(error);
    }
    results.push(result);
    console.log('ANCHOR_OPT_CANDIDATE=' + JSON.stringify(result));
  }
} finally {
  if (browser) await browser.close();
  vite.kill('SIGTERM');
}

const payload = {
  generatedAt: new Date().toISOString(),
  commit: process.env.GITHUB_SHA || null,
  args,
  assumptions: {
    physicalAnchorLimitMeters: 2,
    attachmentRadiusMeters: 0.2,
    spoolToAnchorCenterMeters: 0.12,
    spoolRadiusMeters: 0.03,
    cableStiffnessNPerM: 20000,
    cableDamping: 5,
    minForcePerCableN: 3,
    lineLayering: false,
    closedLoopMotors: false,
    playbackTimeScale: 16,
    gcode: args.pattern === 'bigger' ? 'draw_squares_bigger.gcode' : 'draw_squares.gcode',
  },
  results,
};
const outputPath = path.join(RESULT_DIR, `sweep_${args.mode}_n${args.count}_${args.pattern}${args.smoke ? '_smoke' : ''}.json`);
await writeFile(outputPath, JSON.stringify(payload, null, 2));
console.log('ANCHOR_OPT_SWEEP=' + JSON.stringify(payload));
