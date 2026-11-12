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
  daylight: {
    label: 'Daylight',
    backdrop: '#c4c6c8',
    floor: '#cfd0d2',
    line: '#b5475a',
    slack: '#b06a00',
    frame: '#454a52',
    frameTint: 0.25,
    rotor: '#50555d',
    rotorBack: '#e4e6e9',
    eyelet: '#5d626a',
    axes: ['#b34b3e', '#2f8a46', '#3d6fb3'],
    axesOpacity: 0.6,
    print: '#6b4500',
    printShading: 0.35,
  },
  contrast: {
    label: 'High Contrast',
    backdrop: '#000000',
    floor: '#0d0d0d',
    line: '#ffffff',
    slack: '#ffb000',
    frame: '#8fd0ff',
    frameTint: 0.0,
    rotor: '#ffffff',
    rotorBack: '#3a3a3a',
    eyelet: '#ffffff',
    axes: ['#ff6b5c', '#5cff7a', '#5ca8ff'],
    axesOpacity: 0.7,
    print: '#ffe600',
    printShading: 0.25,
  },
});

export const DEFAULT_COLOR_THEME = 'classic';

export function resolveColorTheme(id) {
  return COLOR_THEMES[id] ?? COLOR_THEMES[DEFAULT_COLOR_THEME];
}
