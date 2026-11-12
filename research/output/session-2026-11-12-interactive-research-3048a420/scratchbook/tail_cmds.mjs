import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const root = new URL('../../../../', import.meta.url);
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const p = new RrfCanPlayer(); p.setDt(.002); p.setAsapMode(true);
p.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4_w_line_layers.g', root), 'utf8')));
const c = []; p.sendCommand = async x => { c.push(x); };
await p.run(Readable.toWeb(fs.createReadStream(new URL('public/RRF_CAN_commands/Hangprinter_logo6_hp4_w_line_layers.can', root))), FileFormat.RRF_CAN_BINARY);
let lastPos = -1; const types = {};
c.forEach((x, i) => { types[x.type] = (types[x.type]||0)+1; if ((x.E||0) > 1e-9) lastPos = i; });
console.log('types', types, 'last +E index', lastPos, 'of', c.length);
const sum = {A:0,B:0,C:0,D:0,E:0}; let n=0;
for (let i = lastPos+1; i < c.length; i++) { for (const k in sum) sum[k] += c[i][k]||0; n++; }
console.log('after last +E:', n, 'ticks; summed rad', sum);
// segments with motion after last E
let runs=[]; let cur=null;
for (let i = lastPos+1; i < c.length; i++) { const m = Math.abs(c[i].A||0)+Math.abs(c[i].B||0)+Math.abs(c[i].C||0)+Math.abs(c[i].D||0); const mv = m>1e-9; if(!cur||cur.mv!==mv){cur={mv,start:i,n:0};runs.push(cur);} cur.n++; }
console.log(runs.slice(0,20));
// peak per-tick rates in tail vs print
const peak = (a,b)=>{let m={A:0,B:0,C:0,D:0}; for(let i=a;i<b;i++) for(const k in m) m[k]=Math.max(m[k],Math.abs(c[i][k]||0)); return m;};
console.log('peak per-tick rad, print', peak(0,lastPos), 'tail', peak(lastPos+1,c.length));
for (let i = lastPos-5; i < c.length; i++) console.log(i, ['A','B','C','D','E'].map(k=>(c[i][k]||0).toFixed(4)).join(' '));
