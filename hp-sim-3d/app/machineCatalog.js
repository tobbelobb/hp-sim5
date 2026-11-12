import { SCENES } from '../../machines/catalog.js';

const PUBLIC_BASE_URL = import.meta.env.BASE_URL || '/';

export const HP4_USDA_KEY = 'hp4.usda';

export function createUsdaCatalog({ baseUrl = PUBLIC_BASE_URL } = {}) {
  return new Map(
    Object.entries(SCENES).map(([scene, { name }]) => {
      const file = scene.split('/').pop();
      return [
        file,
        {
          file,
          label: name,
          url: buildRepositoryFileUrl(scene, { baseUrl }),
          tintColor: null,
          tintColorLoaded: false,
          tintColorPromise: null,
          sourceText: null,
          sourcePromise: null,
        },
      ];
    })
  );
}

export function buildPublicProjectUrl(relativePath, { baseUrl = PUBLIC_BASE_URL } = {}) {
  const normalizedBase = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;
  return `${normalizedBase}${relativePath.replace(/^\/+/, '')}`;
}

// Catalogue paths are repository-relative; files under public/ are served from the base URL.
export function buildRepositoryFileUrl(repositoryPath, options) {
  return buildPublicProjectUrl(repositoryPath.replace(/^public\//, ''), options);
}
