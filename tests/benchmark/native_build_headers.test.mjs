import test from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../../', import.meta.url));
const bundled = path.join(root, 'src/wasm/cable_solver/native/node-api');
const upstream = JSON.parse(fs.readFileSync(path.join(bundled, 'UPSTREAM.json')));

function directory(t) {
  const value = fs.mkdtempSync(path.join(os.tmpdir(), 'hp-sim5 native headers '));
  t.after(() => fs.rmSync(value, { recursive: true, force: true }));
  return value;
}

function build(output, overrides = {}) {
  const env = { ...process.env, ...overrides };
  // Simulate a new chat with no shell-provided header settings.
  for (const key of ['NODE_INCLUDE_DIR', 'CPATH', 'C_INCLUDE_PATH', 'CPLUS_INCLUDE_PATH']) {
    if (!Object.hasOwn(overrides, key)) delete env[key];
  }
  return spawnSync(process.execPath, ['scripts/build_native_physics.mjs', '--no-pgo', '--output', output],
    { cwd: root, env, encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'] });
}

test('native build uses the complete bundled SDK without header environment settings', t => {
  const output = path.join(directory(t), 'physics.node');
  const result = build(output);
  assert.ifError(result.error);
  assert.equal(result.status, 0, result.stdout + result.stderr);
  const metadata = JSON.parse(fs.readFileSync(`${output}.json`));
  assert.equal(metadata.nodeApiHeaders.directory, bundled);
  assert.deepEqual(metadata.nodeApiHeaders.sha256, upstream.headers);
  for (const [file, hash] of Object.entries(upstream.headers)) {
    assert.equal(metadata.sources[`native/node-api/${file}`], hash,
      'Bundled headers must participate in the default addon freshness check');
  }
  const addon = createRequire(import.meta.url)(output);
  const host = addon.create(new Float64Array(), new Float64Array(), 0);
  host.step(.002);
  host.dispose();
});

test('NODE_INCLUDE_DIR remains an explicit override and supports paths with spaces', t => {
  const folder = directory(t);
  const headers = path.join(folder, 'custom headers');
  fs.cpSync(bundled, headers, { recursive: true });
  const output = path.join(folder, 'physics.node');
  const result = build(output, { NODE_INCLUDE_DIR: headers });
  assert.ifError(result.error);
  assert.equal(result.status, 0, result.stdout + result.stderr);
  const metadata = JSON.parse(fs.readFileSync(`${output}.json`));
  assert.equal(metadata.nodeApiHeaders.directory, headers);
  assert.deepEqual(metadata.nodeApiHeaders.sha256, upstream.headers);
});

test('an incomplete explicit SDK fails before compilation with the missing filenames', t => {
  const folder = directory(t);
  const result = build(path.join(folder, 'physics.node'), { NODE_INCLUDE_DIR: folder });
  assert.ifError(result.error);
  assert.notEqual(result.status, 0);
  assert(result.stderr.includes(`Node-API headers are incomplete in ${folder}`));
  for (const file of Object.keys(upstream.headers)) assert(result.stderr.includes(file));
  assert(!fs.existsSync(path.join(folder, 'physics.node')));
});
