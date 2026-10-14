import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';

const root = fileURLToPath(new URL('../', import.meta.url));
const { values: args } = parseArgs({ options: {
  profile: { type: 'string', default: 'release' }, output: { type: 'string' },
} });
if (!['release', 'profiling', 'dev'].includes(args.profile)) throw new Error('Expected --profile release, profiling or dev');
if (!['linux', 'darwin'].includes(process.platform)) throw new Error('Native addon build currently supports Linux and macOS');
const crate = path.join(root, 'src/wasm/cable_solver');
const headers = [process.env.NODE_INCLUDE_DIR, '/usr/include/node', '/usr/local/include/node',
  path.resolve(process.execPath, '../../include/node')].filter(Boolean).find(dir => fs.existsSync(path.join(dir, 'node_api.h')));
if (!headers) throw new Error('Node-API headers are missing; install Node development headers or set NODE_INCLUDE_DIR');
const rustc = execFileSync('rustc', ['--version', '--verbose'], { encoding: 'utf8' }).trim();
const target = rustc.match(/^host: (.+)$/m)?.[1];
if (!target) throw new Error('Cannot determine the native Rust host target');
execFileSync('cargo', ['build', '--manifest-path', path.join(crate, 'Cargo.toml'), '--locked', '--profile', args.profile, '--target', target],
  { cwd: root, stdio: 'inherit', env: { ...process.env, CARGO_TARGET_DIR: path.join(crate, 'target') } });
const destination = path.resolve(args.output ?? path.join(crate, 'target/native/physics.node'));
fs.mkdirSync(path.dirname(destination), { recursive: true });
const library = path.join(crate, 'target', target, args.profile === 'dev' ? 'debug' : args.profile, 'libhp_sim5_cable_solver.a');
const temporary = `${destination}.${process.pid}.tmp`;
execFileSync(process.env.CC ?? 'cc', ['-std=c11', '-Wall', '-Wextra', '-Werror', '-fPIC',
  ...(args.profile === 'dev' ? ['-O0', '-g'] : args.profile === 'profiling' ? ['-O3', '-g'] : ['-O3']),
  '-shared', '-I', headers, path.join(crate, 'native/node_bridge.c'), library,
  ...(process.platform === 'darwin' ? ['-undefined', 'dynamic_lookup'] : ['-ldl', '-lpthread']), '-lm', '-o', temporary],
  { cwd: root, stdio: 'inherit' });
fs.renameSync(temporary, destination);
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const files = ['Cargo.toml', 'Cargo.lock', 'native/node_bridge.c', ...fs.readdirSync(path.join(crate, 'src')).filter(f => f.endsWith('.rs')).map(f => `src/${f}`)];
fs.writeFileSync(`${destination}.json`, JSON.stringify({ profile: args.profile, platform: process.platform, arch: process.arch,
  nodeApi: 8, artifactSha256: sha(fs.readFileSync(destination)),
  sources: Object.fromEntries(files.map(file => [file, sha(fs.readFileSync(path.join(crate, file)))])),
  rustc, target, rustflags: process.env.CARGO_ENCODED_RUSTFLAGS ?? process.env.RUSTFLAGS ?? process.env.CARGO_BUILD_RUSTFLAGS ?? null,
  compiler: process.env.CC ?? 'cc', buildScriptSha256: sha(fs.readFileSync(fileURLToPath(import.meta.url))),
}, null, 2) + '\n');
console.log(`Built native Rust physics (${args.profile}): ${destination}`);
