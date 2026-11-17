// Exploratory: harness sanity + first edge cases. Not a deliverable repro.
import { fixed, mass, spool, pathRecord, world, pos, ATTACHMENT, ROLLING,
  nonFinitePath, nonFiniteEntities } from '../lib.mjs';

const P = (x, y) => [x, y, 0];

// (1) Sanity: hanging mass, one segment, EA = 20000, m = 10.
{
  const es = [fixed(0, P(0, 0)), mass(1, P(0, -1), 10)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 20000, iterations: 8 })]);
  for (let i = 0; i < 400; i++) w.step(0.001);
  const y = pos(w, 1)[1];
  const L = Math.abs(y);
  const expected = 1 + 10 * 9.81 / 20000; // k = EA/L0
  console.log('sanity taut: y', y.toFixed(9), 'expected', (-expected).toFixed(9),
    'tension', w.tension(0, 0).toFixed(4), 'mg', (10 * 9.81).toFixed(4));
}
// (2) Slack: mass resting at rest length.
{
  const es = [fixed(0, P(0, 0)), mass(1, P(0, -1), 10)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1.05, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 20000, iterations: 8 })]);
  w.step(0.001);
  console.log('sanity slack: tension', w.tension(0, 0), 'y', pos(w, 1)[1], 'rest', w.rest(0, 0));
}
// (3) Zero spool radius, cable wrapped 90 deg over spool at [-1,0].
{
  const es = [fixed(0, P(-1, 0)), spool(1, P(0, 0), 0, { mu: 0, inertia: 0.01 }),
    fixed(2, P(0, -1), 0)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ROLLING, cw: true, stored: 0 },
      { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(-1, 0), pb: P(0, 0) },
      { a: 1, b: 2, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 1e5, iterations: 8 })]);
  for (let i = 0; i < 5; i++) w.step(0.001);
  console.log('r=0 spool: nfEnt', nonFiniteEntities(w), 'tensions',
    w.tension(0, 0), w.tension(0, 1), 'pos1', pos(w, 1));
}
// (4) Anchor exactly on spool surface (r = 0.05, point at distance 0.05).
{
  const es = [fixed(0, P(-1, 0)), spool(1, P(0, 0), 0.05), fixed(2, P(0.05, 0), 0)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ROLLING, cw: true, stored: 0 },
      { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(-1, 0), pb: P(0.05, 0) },
      { a: 1, b: 2, rest: 0.05, pa: P(0.05, 0), pb: P(0.05, 0) }],
    { stiffness: 1e5, iterations: 8 })]);
  for (let i = 0; i < 5; i++) w.step(0.001);
  console.log('anchor on surface: rest0', w.rest(0, 0), 'rest1', w.rest(0, 1),
    'tensions', w.tension(0, 0), w.tension(0, 1), 'stored', w.stored(0, 1));
}
// (5) Coincident anchors, rest 0, duplicate points, one segment.
{
  const es = [fixed(0, P(0, 0)), fixed(1, P(0, 0))];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 0, pa: P(0, 0), pb: P(0, 0) }],
    { stiffness: 1e5, iterations: 8 })]);
  for (let i = 0; i < 5; i++) w.step(0.001);
  console.log('coincident rest0: rest', w.rest(0, 0), 'tension', w.tension(0, 0),
    'pos', pos(w, 0), pos(w, 1));
}
// (6) Coincident anchors, positive rest.
{
  const es = [fixed(0, P(0, 0)), fixed(1, P(0, 0))];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 0.5, pa: P(0, 0), pb: P(0, 0) }],
    { stiffness: 1e5, iterations: 8 })]);
  for (let i = 0; i < 5; i++) w.step(0.001);
  console.log('coincident rest .5: rest', w.rest(0, 0), 'tension', w.tension(0, 0));
}
// (7) Negative rest length.
{
  const es = [fixed(0, P(0, 0)), mass(1, P(0, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: -1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 20000, iterations: 8 })]);
  w.step(0.001);
  console.log('negative rest: rest', w.rest(0, 0), 'tension', w.tension(0, 0),
    'y', pos(w, 1)[1]);
}
