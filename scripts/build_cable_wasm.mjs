import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../', import.meta.url));
execFileSync('cargo', ['build', '--manifest-path', 'src/wasm/cable_solver/Cargo.toml',
  '--release', '--target', 'wasm32-unknown-unknown', '--locked'], { cwd: root, stdio: 'inherit' });
const destination = new URL('../public/wasm/cable_solver.wasm', import.meta.url);
fs.mkdirSync(fileURLToPath(new URL('.', destination)), { recursive: true });
fs.copyFileSync(new URL('../src/wasm/cable_solver/target/wasm32-unknown-unknown/release/hp_sim5_cable_solver.wasm', import.meta.url), destination);
console.log(`Built ${fs.statSync(destination).size} byte cable solver: ${fileURLToPath(destination)}`);
