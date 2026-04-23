import fs from 'fs';
import path from 'path';

import { bakeCableSceneUsdaSource } from '../src/js/usd/cable_scene_baker.js';

function resolveOutputPath(inputPath, options = {}) {
  const parsed = path.parse(inputPath);
  const suffix = options.noLineLayering ? '_generated_no_line_layering' : '_generated';
  return path.join(parsed.dir, `${parsed.name}${suffix}${parsed.ext || '.usda'}`);
}

function printUsageAndExit() {
  console.error('Usage: node scripts/bake_cable_scene.mjs [--no-line-layering] <scene.usda> [output.usda]');
  process.exit(1);
}

const args = process.argv.slice(2);
let noLineLayering = false;
const positional = [];
for (const arg of args) {
  if (arg === '--no-line-layering') {
    noLineLayering = true;
  } else {
    positional.push(arg);
  }
}

const [inputPath, explicitOutputPath] = positional;

if (!inputPath) {
  printUsageAndExit();
}

const source = fs.readFileSync(inputPath, 'utf8');
const baked = bakeCableSceneUsdaSource(
  source,
  noLineLayering ? { cablePathHalfWidthOverride: 0.0, deriveAll: true } : {}
);
const outputPath = explicitOutputPath || resolveOutputPath(inputPath, { noLineLayering });

fs.writeFileSync(outputPath, baked.source, 'utf8');
console.log(outputPath);
