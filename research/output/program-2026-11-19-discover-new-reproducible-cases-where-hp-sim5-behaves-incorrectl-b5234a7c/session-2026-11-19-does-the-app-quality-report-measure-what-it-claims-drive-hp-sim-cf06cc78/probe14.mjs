// Probe 14: render the shipped HUD HTML for known inputs (stub DOM).
import { QualityMonitor, polylineSegments, samplePolyline, feed, metricsOf, MM } from './harness.mjs';

function stubElement() {
  const classes = new Set();
  return {
    innerHTML: '',
    textContent: '',
    style: { setProperty() {} },
    classList: {
      add: (c) => classes.add(c),
      remove: (c) => classes.delete(c),
      contains: (c) => classes.has(c),
      toggle: (c, on) => { if (on) classes.add(c); else classes.delete(c); },
    },
    addEventListener() {},
    removeEventListener() {},
  };
}

function hud(name, segs, samples) {
  const el = stubElement();
  const m = new QualityMonitor({ hudElement: el });
  m.setMachineContext({ id: 'hp4', label: 'HP4' });
  m.setReferenceSegments(segs);
  for (const s of samples) m.recordExtrusion({ pos: s.pos, length: s.E, machineId: 'hp4' });
  m.runFinalCheck();
  m.qualityDetailsExpanded = true;
  m.refreshHud(true);
  const text = el.innerHTML.replace(/<[^>]+>/g, '|').replace(/\|+/g, ' | ').trim();
  console.log(`\n== ${name}\n${text}\n`);
  return m.getMetrics();
}

const L = 0.2;
const segs = polylineSegments([[0, 0, 0], [L, 0, 0]]);
hud('0.3 mm rigid offset, 400 samples', segs,
  samplePolyline(segs, 400, { place: 0, offset: () => [0, 0.3 * MM, 0] }));
hud('perfect', segs, samplePolyline(segs, 400, { place: 0 }));
hud('1 mm outlier', (() => {
  const s = samplePolyline(segs, 400, { place: 0 });
  s[200].pos[2] = 1 * MM;
  return s;
})(), (() => {
  const s = samplePolyline(segs, 400, { place: 0 });
  s[200].pos[2] = 1 * MM;
  return s;
})());

const sq = polylineSegments([[0, 0, 0], [0.05, 0, 0], [0.05, 0.05, 0], [0, 0.05, 0], [0, 0, 0]]);
hud('perfect closed square (3 corners counted)', sq, samplePolyline(sq, 100, { place: 0 }));
