import test from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';

// Complete catalogue squares through the real compiled firmware streams, with
// browser defaults (open-loop motors, one substep, catalogue iterations).
for (const machine of ['hp5', 'hp5_pulley_reduced']) {
  for (const firmware of ['rrf', 'klipper']) {
    for (const solver of ['native', 'wasm']) test(`${machine} ${firmware} ${solver} prints squares`, () => {
      const output = execFileSync(process.execPath, ['tests/benchmark/logo_accuracy.mjs',
        '--machine', machine, '--firmware', firmware, '--solver', solver,
        '--gcode', 'draw_squares_bigger', '--open-loop'], { encoding: 'utf8', timeout: 120000 });
      const result = JSON.parse(output.trim().split('\n').at(-1));
      assert.equal(result.substeps, 1);
      assert.equal(result.iterations, machine === 'hp5' ? 1 : 10);
      assert.equal(result.closedLoop, false);
      assert.equal(result.motors.totalMissedSteps, 0);
      assert.deepEqual(result.motors.motors.map(motor => motor.axis), [...'ABCDI']);
      assert(result.metrics.progress > .999);
      assert(result.metrics.trackingRms < .0004, JSON.stringify(result.metrics));
      assert(result.metrics.contourMax < .0005, JSON.stringify(result.metrics));
      console.log(JSON.stringify({ machine, firmware, solver, wallSeconds: result.wallSeconds,
        trackingRmsMm: result.metrics.trackingRms * 1000, contourRmsMm: result.metrics.contourRms * 1000 }));
    });
  }
}
