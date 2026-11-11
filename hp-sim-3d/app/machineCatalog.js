const PUBLIC_BASE_URL = import.meta.env.BASE_URL || '/';

export const HP3_USDA_KEY = 'hp3_rigid_body.usda';
export const HP4_USDA_KEY = 'hp4_rigid_body.usda';
export const FOUR_HIGH_ANCHORS_USDA_KEY = 'four_high_anchors_rigid_body.usda';
export const CUBECORNERS_USDA_KEY = 'cubecorners_rigid_body.usda';

export const AVAILABLE_USDAS = Object.freeze([
  { file: HP4_USDA_KEY, label: 'Hangprinter v4 (default)' },
  { file: HP3_USDA_KEY, label: 'Hangprinter v3' },
  { file: FOUR_HIGH_ANCHORS_USDA_KEY, label: 'Four High Anchors' },
  { file: CUBECORNERS_USDA_KEY, label: 'CubeCorners' },
  { file: 'slideprinter_multi_unit_rigid_body.usda', label: 'Slideprinter Multi Unit' },
  { file: 'slideprinter_rigid_body.usda', label: 'Slideprinter Original' },
  { file: 'slideprinter_hexagon_rigid_body.usda', label: 'Slideprinter (hexagon)' },
  { file: 'slideprinter_single_pinholes_rigid_body.usda', label: 'Slideprinter (single pinholes)' },
]);

export const RRF_CONFIG_VARIANTS_BY_USDA_KEY = Object.freeze({
  [HP3_USDA_KEY]: Object.freeze({
    default: 'RRF/run/vsd/sys/config_hp3.g',
    lineLayered: 'RRF/run/vsd/sys/config_hp3_w_line_layers.g',
  }),
  [HP4_USDA_KEY]: Object.freeze({
    default: 'RRF/run/vsd/sys/config_hp4.g',
    lineLayered: 'RRF/run/vsd/sys/config_hp4_w_line_layers.g',
  }),
  [FOUR_HIGH_ANCHORS_USDA_KEY]: Object.freeze({
    default: 'RRF/run/vsd/sys/config_skycam.g',
    lineLayered: 'RRF/run/vsd/sys/config_skycam_w_line_layers.g',
  }),
  [CUBECORNERS_USDA_KEY]: Object.freeze({
    default: 'RRF/run/vsd/sys/config_cubecorners.g',
    lineLayered: 'RRF/run/vsd/sys/config_cubecorners_w_line_layers.g',
  }),
});

export function createUsdaCatalog({ baseUrl = PUBLIC_BASE_URL } = {}) {
  const normalizedBase = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;
  return new Map(
    AVAILABLE_USDAS.map((entry) => [
      entry.file,
      {
        ...entry,
        url: `${normalizedBase}usd_scenes/${entry.file}`,
        tintColor: null,
        tintColorLoaded: false,
        tintColorPromise: null,
        sourceText: null,
        sourcePromise: null,
      },
    ])
  );
}

export function buildPublicProjectUrl(relativePath, { baseUrl = PUBLIC_BASE_URL } = {}) {
  const normalizedBase = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;
  return `${normalizedBase}${relativePath.replace(/^\/+/, '')}`;
}

