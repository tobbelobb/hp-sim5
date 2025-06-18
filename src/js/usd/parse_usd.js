export async function parseUsda(url) {
  const res = await fetch(url);
  if (!res.ok) {
    throw new Error(`Failed to fetch ${url}: ${res.status}`);
  }
  const text = await res.text();
  return parseUsdaText(text);
}

export function parseUsdaText(text) {
  const lines = text.split(/\r?\n/);
  const entities = {};
  const joints = [];
  const paths = [];
  let i = 0;
  while (i < lines.length) {
    let line = lines[i].trim();
    if (line.startsWith('def ') && line.includes('"')) {
      const name = line.split('"')[1];
      if (line.trim().endsWith('{')) { i++; continue; }
      let purpose = null;
      i++;
      while (i < lines.length) {
        const l = lines[i].trim();
        const m = l.match(/purpose\s*=\s*"([^"]+)"/);
        if (m) purpose = m[1];
        if (l.endsWith(')')) { i++; break; }
        i++;
      }
      const attrs = {};
      if (i < lines.length && lines[i].trim() === '{') {
        i++;
        while (i < lines.length) {
          const l = lines[i].trim();
          if (l === '}') break;
          let m;
          if (l.includes('xformOp:translate')) {
            m = l.match(/\(([^)]+)\)/);
            if (m) {
              attrs.pos = m[1].split(',').map(n => parseFloat(n)).slice(0,2);
            }
          }
          const numAttrs = ['radius','mass','angVel','velX','velY','restLength'];
          for (const a of numAttrs) {
            if (l.startsWith(`double ${a}`)) {
              attrs[a] = parseFloat(l.split('=')[1]);
            }
          }
          if (l.startsWith('token entityA')) attrs.entityA = l.split('=')[1].trim().replace(/"/g,'');
          if (l.startsWith('token entityB')) attrs.entityB = l.split('=')[1].trim().replace(/"/g,'');
          if (l.startsWith('double3 attachA')) {
            m = l.match(/\(([^)]+)\)/);
            if (m) attrs.attachA = m[1].split(',').map(n => parseFloat(n));
          }
          if (l.startsWith('double3 attachB')) {
            m = l.match(/\(([^)]+)\)/);
            if (m) attrs.attachB = m[1].split(',').map(n => parseFloat(n));
          }
          if (l.startsWith('token[] joints')) {
            m = l.match(/\[([^\]]+)\]/);
            if (m) attrs.joints = m[1].split(',').map(s => s.trim().replace(/"/g,''));
          }
          if (l.startsWith('token[] linkTypes')) {
            m = l.match(/\[([^\]]+)\]/);
            if (m) attrs.linkTypes = m[1].split(',').map(s => s.trim().replace(/"/g,''));
          }
          if (l.startsWith('bool[] cw')) {
            m = l.match(/\[([^\]]+)\]/);
            if (m) attrs.cw = m[1].split(',').map(s => s.trim().toLowerCase() === 'true');
          }
          if (l.startsWith('double[] stored')) {
            m = l.match(/\[([^\]]+)\]/);
            if (m) attrs.stored = m[1].split(',').map(s => parseFloat(s));
          }
          i++;
        }
        if (i < lines.length && lines[i].trim() === '}') i++;
      }
      if (purpose === 'cable_joint') {
        joints.push({ name, ...attrs });
      } else if (purpose === 'cable_path') {
        paths.push({ name, ...attrs });
      } else if (purpose) {
        entities[name] = { name, type: purpose, ...attrs };
      }
    } else {
      i++;
    }
  }
  return { entities, joints, paths };
}
