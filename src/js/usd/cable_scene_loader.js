import { OpenText, readUsdaSource } from './stage.js';
import { bakeCableSceneUsdaSource } from './cable_scene_baker.js';

export async function OpenCableScene(pathOrSource, options = {}) {
  const sourceText = await readUsdaSource(pathOrSource);
  const baked = bakeCableSceneUsdaSource(sourceText, options);
  return OpenText(baked.source);
}
