// Regenerate measurements.json from the shipped worker.
//   node make_measurements.mjs
import { writeFileSync } from 'node:fs';
import { execSync } from 'node:child_process';
import { measure, CONFIG, SPOOL_R, REPO } from './harness.mjs';

const DT = 0.002; // hp4.usda timeCodesPerSecond = 500

const findings = [
  {
    id: 'F1',
    title: 'G91 relative positioning silently ignored',
    classification: 'a (shipped, user-visible: reference overlay and simulated motion disagree)',
    code: [
      'hp-sim-3d/app/workers/moveCommander.js:209-224 (only G1/G6/G92/M82/M83 recognised)',
      'hp-sim-3d/app/workers/moveCommander.js:229-234 (X/Y/Z always absolute)',
      'hp-sim-3d/app/inspectionToolsController.js:96-103 (reference parser DOES implement G91)',
    ],
    firmware: ['RRF/ReprapFirmware/src/GCodes/GCodes2.cpp:573-576', 'klipper/klippy/extras/gcode_move.py:174-176'],
    repro: 'repro_g91.mjs',
    gcode: 'M82\nG91\nG1 X10 Y0 E1 F600\nG1 X10 Y0 E2 F600\n',
  },
  {
    id: 'F2',
    title: 'G0 rapid lines dropped (and F on a G0 line is lost)',
    classification: 'a (shipped, user-visible: reference overlay and simulated path disagree)',
    code: [
      'hp-sim-3d/app/workers/moveCommander.js:214 (only literal "G1" is a move)',
      'hp-sim-3d/app/workers/moveCommander.js:222-224 and 330 (position bookkeeping skipped)',
      'hp-sim-3d/app/workers/moveCommander.js:124-128 (F only read on G1 lines)',
      'hp-sim-3d/app/inspectionToolsController.js:124 (reference parser treats G0 == G1)',
    ],
    firmware: ['RRF/ReprapFirmware/src/GCodes/GCodes2.cpp:230-233', "klipper/klippy/gcode.py:46 (G0 -> cmd_G1)"],
    repro: 'repro_g0.mjs',
    gcode: 'M82\nG0 X10 Y0 F9000\nG1 X10 Y10 E1 F600\nG0 X20 Y10 F9000\nG1 X20 Y20 E2 F600\n',
    extra: {
      label: 'F on a G0 line is lost',
      gcode: 'M82\nG0 F9000\nG1 X10 Y0 E1\n',
    },
  },
  {
    id: 'F3',
    title: 'Numeric spellings G00/G01/G02/G03 dropped',
    classification: 'a (shipped, user-visible: reference overlay and simulated motion disagree)',
    code: [
      'hp-sim-3d/app/workers/moveCommander.js:209-220 (string comparison with "G1"/"G6"/"G92")',
      'hp-sim-3d/app/inspectionToolsController.js:84-86,124 (parseInt command number, so G01 == G1)',
    ],
    firmware: ['RRF/ReprapFirmware/src/GCodes/GCodeBuffer/StringParser.cpp:985-1032', 'Klipper rejects G01 (handler keys are "G1") - limitation'],
    repro: 'repro_g01.mjs',
    gcode: 'M82\nG01 X10 Y0 E1 F600\nG01 X20 Y0 E2 F600\n',
    extra: {
      label: 'G00 rapid also dropped -> next G1 cuts the diagonal',
      gcode: 'M82\nG00 X10 Y0 F9000\nG1 X10 Y10 E1 F600\n',
    },
  },
  {
    id: 'F4',
    title: 'Lines with N<line number> (and *checksum) dropped wholesale',
    classification: 'a (shipped, wrong motion, but rare in uploaded files)',
    code: [
      'hp-sim-3d/app/workers/moveCommander.js:209-224 (first token "N10" is not a command)',
      'hp-sim-3d/app/inspectionToolsController.js:84-124 (same; reference overlay empty too)',
    ],
    firmware: [
      'RRF/ReprapFirmware/src/GCodes/GCodeBuffer/StringParser.cpp:123-141 (N prefix), 186-192 / 272-285 (checksum)',
      'klipper/klippy/gcode.py:199-205 (line number skipped)',
    ],
    repro: 'repro_line_numbers.mjs',
    gcode: 'M82\nN10 G1 X10 Y0 E1 F600*45\nN20 G1 X20 Y0 E2 F600*46\n',
  },
  {
    id: 'F5',
    title: 'G20 inches interpreted as millimetres (25.4x under-travel)',
    classification: 'a (shipped, wrong motion; Klipper instead raises an error)',
    code: [
      'hp-sim-3d/app/workers/moveCommander.js:222-224 (G20 skipped)',
      'hp-sim-3d/app/inspectionToolsController.js:88-126 (no G20 case either)',
    ],
    firmware: [
      'RRF/ReprapFirmware/src/GCodes/GCodes2.cpp:376-380 (UseInches)',
      'RRF/ReprapFirmware/src/GCodes/GCodeBuffer/GCodeBuffer.cpp:1013-1016 (x25.4)',
      'klipper/klippy/extras/gcode_move.py:159-161 (explicit error)',
    ],
    repro: 'repro_g20.mjs',
    gcode: 'M82\nG20\nG1 X1 Y0 E1 F600\nG1 X1.5 Y0 E2 F600\n',
  },
  {
    id: 'F6',
    title: 'G2/G3 arcs dropped (travel and extrusion lost)',
    classification: 'a (shipped, wrong motion; overlay agrees with the sim, both disagree with the machine)',
    code: [
      'hp-sim-3d/app/workers/moveCommander.js:214-224 (G2/G3 not moves)',
      'hp-sim-3d/app/workers/moveCommander.js:236-240 (arc E never reaches last_e)',
      'hp-sim-3d/app/inspectionToolsController.js:124 (reference parser also skips G2/G3)',
    ],
    firmware: ['RRF/ReprapFirmware/src/GCodes/GCodes2.cpp:251-277 (DoArcMove)', 'klipper/klippy/extras/gcode_arcs.py:36-37'],
    repro: 'repro_arcs.mjs',
    gcode: 'M82\nG1 X10 Y0 E1 F600\nG2 X20 Y10 I10 J0 E2 F600\n',
    extra: {
      label: 'mid-file half circle G3',
      gcode: 'M82\nG3 X20 Y0 I10 J0 E2 F600\nG1 X20 Y20 E3 F600\n',
    },
  },
  {
    id: 'F7',
    title: 'G4 dwell contributes no simulated time',
    classification: 'b (silent unsupported command; timing only, no wrong motion)',
    code: [
      'hp-sim-3d/app/workers/moveCommander.js:209-224 (G4 skipped)',
      'hp-sim-3d/app/remoteSpoolSystem.js:439-449 (one command = one physics step = one dt of simulated time)',
    ],
    firmware: ['RRF/ReprapFirmware/src/GCodes/GCodes.cpp:3742-3790', 'klipper/klippy/toolhead.py:554-557'],
    repro: 'repro_dwell.mjs',
    gcode: 'M82\nG1 X10 Y0 E1 F600\nG4 P1000\nG1 X20 Y0 E2 F600\n',
  },
  {
    id: 'F8',
    title: 'Initial modal feed rate 1000 mm/min vs RRF 3000 / Klipper 1500',
    classification: 'b (timing only, no wrong motion)',
    code: ['hp-sim-3d/app/workers/moveCommander.js:32 (last_speed_mm_per_min = 1000.0)'],
    firmware: ['RRF/ReprapFirmware/src/Config/Configuration.h:220 (DefaultFeedRate = 3000.0)', 'klipper/klippy/extras/gcode_move.py:55 (25 mm/s = 1500 mm/min)'],
    repro: 'repro_feedrate_default.mjs',
    gcode: 'M82\nG1 X10 Y0 E1\nG1 X20 Y0 E2\n',
  },
];

const negatives = [
  ['explicit G21 (mm)', 'M82\nG21\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n'],
  ['explicit G90 (absolute)', 'M82\nG90\nG1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n'],
  ['CRLF, no final newline', 'M82\r\nG1 X10 Y0 E1 F600\r\nG1 X20 Y0 E2 F600'],
  ['lowercase', 'm82\ng1 x10 y0 e1 f600\ng1 x20 y0 e2 f600\n'],
  ['tab separators', 'M82\nG1\tX10\tY0\tE1\tF600\nG1\tX20\tY0\tE2\tF600\n'],
  ['checksum without line number', 'M82\nG1 X10 Y0 E1*45 F600\nG1 X20 Y0 E2 F600*46\n'],
  ['semicolon comments', 'M82 ; c\nG1 X10 Y0 E1 F600 ; a\nG1 X20 Y0 E2 F600 ; b\n'],
  ['parenthesised inline comment', 'M82\nG1 X10 Y0 E1 F600 (a)\nG1 X20 Y0 E2 F600(b)\n'],
  ['feedrate-only G1 line', 'M82\nG1 F600\nG1 X10 Y0 E1\nG1 X20 Y0 E2\n'],
  ['two commands on one line (invalid input)', 'M82 G1 X10 Y0 E1 F600\nG1 X20 Y0 E2 F600\n'],
  ['blank lines / comment-only lines', 'M82\n\n;\n   \nG1 X10 Y0 E1 F600\n\nG1 X20 Y0 E2 F600\n'],
];

function pack(m) {
  return {
    dt: m.dt,
    commanded: {
      travelMm: m.commanded.travelMm,
      extrusionMm: m.commanded.extrusionMm,
      timeSec: m.commanded.timeSec,
      endMm: m.commanded.endMm,
    },
    simulated: {
      travelMm: m.sim.pathLengthMm,
      extrusionMm: m.sim.extrusionMm,
      timeSec: m.sim.timeSec,
      endMm: m.sim.endMm,
      emittedCommands: m.sim.steps,
      beliefMm: [m.sim.beliefMm.X, m.sim.beliefMm.Y, m.sim.beliefMm.Z],
      maxPathDeviationMm: m.sim.maxPathDeviationMm,
      endErrorMm: m.sim.endErrorMm,
    },
    referenceOverlay: {
      segments: m.overlay.segments,
      travelMm: m.overlay.travelMm,
      extrusionMm: m.overlay.extrusionMm,
      endMm: m.overlay.endMm,
    },
    errorPct: {
      travel: m.commanded.travelMm === 0 ? null : ((m.sim.pathLengthMm - m.commanded.travelMm) / m.commanded.travelMm) * 100,
      extrusion: m.commanded.extrusionMm === 0 ? null : ((m.sim.extrusionMm - m.commanded.extrusionMm) / m.commanded.extrusionMm) * 100,
      time: m.commanded.timeSec === 0 ? null : ((m.sim.timeSec - m.commanded.timeSec) / m.commanded.timeSec) * 100,
    },
  };
}

const out = {
  investigation: 'session-2026-11-19-which-g-code-program-level-motion-semantics-does-the-browser-upl-1596692d',
  generatedAt: new Date().toISOString(),
  repo: REPO,
  commit: execSync('git rev-parse --short HEAD', { cwd: REPO }).toString().trim(),
  node: process.version,
  sources: {
    worker: 'hp-sim-3d/app/workers/moveCommander.js',
    referenceParser: 'hp-sim-3d/app/inspectionToolsController.js',
    consumer: 'hp-sim-3d/app/remoteSpoolSystem.js',
    // real 64-hex digests, one entry per file
    sha256: execSync(
      'sha256sum hp-sim-3d/app/workers/moveCommander.js hp-sim-3d/app/inspectionToolsController.js hp-sim-3d/app/remoteSpoolSystem.js',
      { cwd: REPO },
    )
      .toString()
      .trim()
      .split('\n')
      .map((l) => {
        const m = /^([0-9a-f]{64})\s+(.+)$/.exec(l.trim());
        if (!m) throw new Error(`unexpected sha256sum line: ${l}`);
        return { path: m[2], sha256: m[1] };
      }),
  },
  method: {
    dtSec: DT,
    dtSource: 'public/usd_scenes/hp4.usda timeCodesPerSecond = 500',
    kinematicsConfig: CONFIG.trim(),
    spoolRadiusMm: SPOOL_R,
    oracle: 'harness.mjs oracle(): RRF/Klipper semantics, constant-feed time model (no acceleration)',
    timeModel: 'simulated time = emitted commands x dt (remoteSpoolSystem consumes one command per physics step)',
    pathModel: 'simulated travel = polyline through per-step effector positions inverted from the emitted motor angles',
  },
  findings: [],
  negatives: [],
};

for (const f of findings) {
  const m = await measure(f.gcode, { dt: DT });
  const rec = {
    id: f.id,
    title: f.title,
    classification: f.classification,
    codePaths: f.code,
    firmwareCitations: f.firmware,
    reproducer: f.repro,
    gcode: f.gcode.split('\n'),
    ...pack(m),
  };
  if (f.extra) {
    const m2 = await measure(f.extra.gcode, { dt: DT });
    rec.subCase = { label: f.extra.label, gcode: f.extra.gcode.split('\n'), ...pack(m2) };
  }
  out.findings.push(rec);
  console.log(`${f.id} packed`);
}

for (const [name, text] of negatives) {
  const m = await measure(text, { dt: DT });
  out.negatives.push({ name, gcode: text.split('\n'), ...pack(m) });
  console.log(`negative packed: ${name}`);
}

writeFileSync(new URL('./measurements.json', import.meta.url), JSON.stringify(out, null, 2));
console.log('\nwrote measurements.json');
