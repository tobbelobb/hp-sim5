import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseArgs } from 'node:util';

const root = fileURLToPath(new URL('../', import.meta.url));
const { values: args } = parseArgs({ options: {
  profile: { type: 'string', default: 'release' }, output: { type: 'string' },
  pgo: { type: 'boolean' },
}, allowNegative: true });
if (!['release', 'profiling', 'dev'].includes(args.profile)) throw new Error('Expected --profile release, profiling or dev');
if (!['linux', 'darwin'].includes(process.platform)) throw new Error('Native addon build currently supports Linux and macOS');
const crate = path.join(root, 'src/wasm/cable_solver');
const headers = [process.env.NODE_INCLUDE_DIR, '/usr/include/node', '/usr/local/include/node',
  path.resolve(process.execPath, '../../include/node')].filter(Boolean).find(dir => fs.existsSync(path.join(dir, 'node_api.h')));
if (!headers) throw new Error('Node-API headers are missing; install Node development headers or set NODE_INCLUDE_DIR');
const rustc = execFileSync('rustc', ['--version', '--verbose'], { encoding: 'utf8' }).trim();
const target = rustc.match(/^host: (.+)$/m)?.[1];
if (!target) throw new Error('Cannot determine the native Rust host target');
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const rustflags = process.env.CARGO_ENCODED_RUSTFLAGS ?? process.env.RUSTFLAGS ?? process.env.CARGO_BUILD_RUSTFLAGS ?? null;
if (args.pgo ?? (args.profile !== 'dev')) {
  if (/profile-(generate|use)/.test(rustflags ?? '')) throw new Error('Use --no-pgo with explicit profile-generate/profile-use flags');
  const libdir = execFileSync('rustc', ['--print', 'target-libdir'], { encoding: 'utf8' }).trim();
  const profdata = path.resolve(libdir, '../bin', process.platform === 'win32' ? 'llvm-profdata.exe' : 'llvm-profdata');
  if (!fs.existsSync(profdata)) throw new Error('PGO needs rustup component add llvm-tools-preview; use --no-pgo for an untrained build');
  const parent = path.join(crate, 'target/native');
  fs.mkdirSync(parent, { recursive: true });
  const directory = fs.mkdtempSync(path.join(parent, 'pgo-'));
  const flags = process.env.CARGO_ENCODED_RUSTFLAGS?.split('\x1f').filter(Boolean)
    ?? rustflags?.trim().split(/\s+/).filter(Boolean) ?? [];
  if (!flags.some(flag => flag.includes('target-cpu'))) flags.push('-C', 'target-cpu=native');
  const build = (flag, output) => execFileSync(process.execPath, [fileURLToPath(import.meta.url),
    '--no-pgo', '--profile', args.profile, '--output', output], { cwd: root, stdio: 'inherit',
    env: { ...process.env, CARGO_ENCODED_RUSTFLAGS: [...flags, '-C', flag].join('\x1f') } });
  const instrumented = path.join(directory, 'instrumented.node');
  build(`profile-generate=${directory}`, instrumented);
  const training = path.join(directory, 'training.json');
  execFileSync(process.execPath, ['scripts/train_native_physics.mjs', instrumented, training], { cwd: root, stdio: 'inherit' });
  const merged = path.join(directory, 'merged.profdata');
  const profiles = fs.readdirSync(directory).filter(name => name.endsWith('.profraw')).map(name => path.join(directory, name));
  if (!profiles.length) throw new Error('Native PGO training produced no profiles');
  execFileSync(profdata, ['merge', '-o', merged, ...profiles], { cwd: root, stdio: 'inherit' });
  const destination = path.resolve(args.output ?? path.join(parent, 'physics.node'));
  build(`profile-use=${merged}`, destination);
  const metadata = JSON.parse(fs.readFileSync(`${destination}.json`));
  metadata.pgo = { profile: merged, sha256: sha(fs.readFileSync(merged)),
    training: JSON.parse(fs.readFileSync(training)),
    trainerSha256: sha(fs.readFileSync(path.join(root, 'scripts/train_native_physics.mjs'))) };
  fs.writeFileSync(`${destination}.json`, JSON.stringify(metadata, null, 2) + '\n');
  console.log(`Built PGO native Rust physics: ${destination}`);
  process.exit(0);
}
execFileSync('cargo', ['build', '--manifest-path', path.join(crate, 'Cargo.toml'), '--locked', '--profile', args.profile, '--target', target],
  { cwd: root, stdio: 'inherit', env: { ...process.env, CARGO_TARGET_DIR: path.join(crate, 'target') } });
const destination = path.resolve(args.output ?? path.join(crate, 'target/native/physics.node'));
fs.mkdirSync(path.dirname(destination), { recursive: true });
const library = path.join(crate, 'target', target, args.profile === 'dev' ? 'debug' : args.profile, 'libhp_sim5_cable_solver.a');
const temporary = `${destination}.${process.pid}.tmp`;
// Rust staticlib archives omit the instrumentation runtime. Force its atexit
// writer into the Node addon so profile-generate actually produces usable data.
const instrumentation = [];
if (/profile-generate/.test(rustflags ?? '')) {
  const libdir = execFileSync('rustc', ['--print', 'target-libdir'], { encoding: 'utf8' }).trim();
  const library = fs.readdirSync(libdir).find(name => /^libprofiler_builtins-.*\.rlib$/.test(name));
  if (!library) throw new Error('Rust profiler_builtins runtime is missing');
  instrumentation.push(`-Wl,-u,${process.platform === 'darwin' ? '_' : ''}__llvm_profile_runtime`, path.join(libdir, library));
}
execFileSync(process.env.CC ?? 'cc', ['-std=c11', '-Wall', '-Wextra', '-Werror', '-fPIC',
  ...(args.profile === 'dev' ? ['-O0', '-g'] : args.profile === 'profiling' ? ['-O3', '-g'] : ['-O3']),
  '-shared', '-I', headers, path.join(crate, 'native/node_bridge.c'), library, ...instrumentation,
  ...(process.platform === 'darwin' ? ['-undefined', 'dynamic_lookup'] : ['-ldl', '-lpthread']), '-lm', '-o', temporary],
  { cwd: root, stdio: 'inherit' });
fs.renameSync(temporary, destination);
const files = ['Cargo.toml', 'Cargo.lock', 'native/node_bridge.c', ...fs.readdirSync(path.join(crate, 'src')).filter(f => f.endsWith('.rs')).map(f => `src/${f}`)];
fs.writeFileSync(`${destination}.json`, JSON.stringify({ profile: args.profile, platform: process.platform, arch: process.arch,
  nodeApi: 8, artifactSha256: sha(fs.readFileSync(destination)),
  sources: Object.fromEntries(files.map(file => [file, sha(fs.readFileSync(path.join(crate, file)))])),
  rustc, target, rustflags,
  compiler: process.env.CC ?? 'cc', buildScriptSha256: sha(fs.readFileSync(fileURLToPath(import.meta.url))),
}, null, 2) + '\n');
console.log(`Built native Rust physics (${args.profile}): ${destination}`);
