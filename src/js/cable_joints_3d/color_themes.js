// Scene colour themes. UI colours for the same theme ids live in
// hp-sim-3d/style.css under :root[data-theme=...].
//
// A null colour means "use the machine's authored colour" (its USD material
// or machine tint), which keeps overlapping machines distinguishable.
//
//   backdrop, floor   studio background and the floor fading into it
//   line, slack       taut cable colour and the optional slack highlight
//   frame             rigid frame tubes; frameTint mixes in the machine tint
//   rotor, rotorBack  bearings/spools and the half that shows their rotation
//   eyelet            pinhole and anchor rings
//   axes, axesOpacity z=0 crosshair colours (x, y, z)
//   print             default printed-material colour (quality colours win)
//   printShading      0 = flat bright beads, 1 = fully shaded round beads

export const COLOR_THEMES = Object.freeze({
  classic: {
    label: 'Classic',
    backdrop: '#1b2b3c',
    floor: '#22344a',
    line: null,
    slack: '#ff9f43',
    frame: null,
    rotor: null,
    rotorBack: '#2a3542',
    eyelet: '#c7cbe0',
    axes: ['#ddddff', '#ddddff', '#ddddff'],
    axesOpacity: 0.18,
    print: null,
    printShading: 0.0,
  },
  graphite: {
    label: 'Graphite',
    backdrop: '#2d3135',
    floor: '#35383d',
    line: '#daa0a0',
    slack: '#dfb274',
    frame: '#c4c8ce',
    frameTint: 0.25,
    rotor: '#c3c7cc',
    rotorBack: '#3a3f47',
    eyelet: '#dfe2e6',
    axes: ['#cd776c', '#69b27a', '#779bc9'],
    axesOpacity: 0.55,
    print: null,
    printShading: 1.0,
  },
  // Warm paper and botanical accents inspired by Everforest Light.
  daylight: {
    label: 'Daylight',
    backdrop: '#f3f0df',
    floor: '#e9e5d2',
    line: '#8b4963',
    slack: '#9b650d',
    frame: '#687b83',
    frameTint: 0.15,
    rotor: '#80918b',
    rotorBack: '#35464b',
    eyelet: '#5b6e75',
    axes: ['#b44c42', '#52733d', '#426f9d'],
    axesOpacity: 0.7,
    print: '#386b70',
    printShading: 0.65,
  },
  contrast: {
    label: 'High Contrast',
    backdrop: '#101419',
    floor: '#192129',
    line: '#f4f1de',
    slack: '#f2c76b',
    frame: '#83d6cf',
    frameTint: 0.0,
    rotor: '#f4f1de',
    rotorBack: '#374854',
    eyelet: '#c5dedb',
    axes: ['#ff8c82', '#a4d88b', '#91bfff'],
    axesOpacity: 0.8,
    print: '#f2c76b',
    printShading: 0.15,
  },
  // Vim palette references: folke/tokyonight.nvim, morhetz/gruvbox, shaunsingh/nord.nvim.
  'tokyo-night': {
    label: 'Tokyo Night',
    backdrop: '#1a1b26',
    floor: '#24283b',
    line: '#7dcfff',
    slack: '#e0af68',
    frame: '#a9b1d6',
    frameTint: 0.15,
    rotor: '#bb9af7',
    rotorBack: '#414868',
    eyelet: '#c0caf5',
    axes: ['#f7768e', '#9ece6a', '#7aa2f7'],
    axesOpacity: 0.65,
    print: '#f7768e',
    printShading: 0.8,
  },
  'gruvbox': {
    label: 'Gruvbox',
    backdrop: '#282828',
    floor: '#32302f',
    line: '#ebdbb2',
    slack: '#fe8019',
    frame: '#a89984',
    frameTint: 0.15,
    rotor: '#83a598',
    rotorBack: '#504945',
    eyelet: '#d5c4a1',
    axes: ['#fb4934', '#b8bb26', '#83a598'],
    axesOpacity: 0.65,
    print: '#fabd2f',
    printShading: 0.8,
  },
  'nord': {
    label: 'Nord',
    backdrop: '#2e3440',
    floor: '#3b4252',
    line: '#88c0d0',
    slack: '#ebcb8b',
    frame: '#d8dee9',
    frameTint: 0.15,
    rotor: '#81a1c1',
    rotorBack: '#434c5e',
    eyelet: '#eceff4',
    axes: ['#bf616a', '#a3be8c', '#81a1c1'],
    axesOpacity: 0.7,
    print: '#b48ead',
    printShading: 0.8,
  },
});

export const DEFAULT_COLOR_THEME = 'graphite';

export function resolveColorTheme(id) {
  return COLOR_THEMES[id] ?? COLOR_THEMES[DEFAULT_COLOR_THEME];
}
