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
});

export const DEFAULT_COLOR_THEME = 'graphite';

export function resolveColorTheme(id) {
  return COLOR_THEMES[id] ?? COLOR_THEMES[DEFAULT_COLOR_THEME];
}
