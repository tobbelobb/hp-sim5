import fs from 'fs';
import path from 'path';

import { bakeCableSceneUsdaSource } from '../src/js/usd/cable_scene_baker.js';

function resolveOutputPath(inputPath) {
  const parsed = path.parse(inputPath);
  return path.join(parsed.dir, `${parsed.name}_generated${parsed.ext || '.usda'}`);
}

function printUsageAndExit() {
  console.error('Usage: node scripts/bake_cable_scene.mjs <scene.usda> [output.usda]');
  process.exit(1);
}

const [, , inputPath, explicitOutputPath] = process.argv;

if (!inputPath) {
  printUsageAndExit();
}

const source = fs.readFileSync(inputPath, 'utf8');
const baked = bakeCableSceneUsdaSource(source);
const outputPath = explicitOutputPath || resolveOutputPath(inputPath);

fs.writeFileSync(outputPath, baked.source, 'utf8');
console.log(outputPath);
