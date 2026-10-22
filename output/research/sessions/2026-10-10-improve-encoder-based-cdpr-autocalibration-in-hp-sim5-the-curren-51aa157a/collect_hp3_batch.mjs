// E04/E05: production HP3 physics and collector, RRF batch-planned H2 moves.
// Independent RRF file jobs replace the unavailable persistent HTTP transport.
import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { spawnSync } from 'node:child_process';
import { pathToFileURL } from 'node:url';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { bakeCableSceneUsdaSource } from '../../../../src/js/usd/cable_scene_baker.js';
import { createExternalCommandController } from '../../../../hp-sim-3d/app/externalCommandSocket.js';
import { RrfHttpBridge } from '../../../../integrations/rrf/rrfHttpBridge.js';
const collectorModule = process.argv[3] ? pathToFileURL(process.argv[3]).href
  : new URL('../../../../autocal/control/behaviors/sweep_data_collection.mjs',import.meta.url).href;
const {collectSweepData} = await import(collectorModule);

const directory = process.argv[2];
fs.mkdirSync(directory,{recursive:true});
const hash = p => createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const files = ['public/usd_scenes/hp3_rigid_body.usda','RRF/run/vsd/sys/config_hp3.g',
  'autocal/control/behaviors/sweep_data_collection.mjs','autocal/control/primitives/uncalibrated_actions.mjs'];
const sourceHashes = Object.fromEntries(files.map(p=>[p,hash(p)]));
const scene = bakeCableSceneUsdaSource(fs.readFileSync(files[0],'utf8'),
  {cablePathHalfWidthOverride:0}).source;
fs.writeFileSync(path.join(directory,'scene-no-layering.usda'),scene);
const {world,remote,dt} = createHeadlessWorld(scene);
let step=0, response, moves=0, reads=0, torqueCommands=0, planningWallMs=0;
const started=performance.now();
const manifest={backend:'production-headless-js/RRF-batch-H2',scene:files[0],dt,
  sourceHashes,collectorModule,baked_scene_sha256:hash(path.join(directory,'scene-no-layering.usda')),
  settings:{closedLoopMotorsEnabled:true,enableLayering:false,cablePathHalfWidthOverride:0},
  limitations:['RRF is initialized per relative motion, rather than persistent HTTP',
  'No firmware encoder requests; production endpoint reports encoders directly',
  'Torque Nm conversion uses known R30, unit gearing and mechanical advantage']};
fs.writeFileSync(path.join(directory,'manifest.json'),JSON.stringify(manifest,null,2));
class Socket {
  static OPEN=1; readyState=1;
  addEventListener(){}
  send(v){response=JSON.parse(v);}
}
const controller=createExternalCommandController({world,url:'ws://unused',WebSocketCtor:Socket,
  commands:{getRemoteSystem:()=>remote,pushExternalCommands(batch){batch.forEach(c=>remote.addCommand(c));return true;}},
  runtime:{resume(){}},logger:{info(){},log(){},warn(){}}});
controller.connect();
controller.handlePayload({type:'simulation_settings',closedLoopMotorsEnabled:true,lineLayeringEnabled:false});
async function advance(ms=0,drain=false){
  const n=Math.max(Math.ceil(ms/1000/dt-1e-9),drain?remote.getQueueLength():0);
  for(let i=0;i<n;i++){
    world.update(dt);step++;
    world.setResource('researchClock',{step,time:step*dt,source:'batch-hp3.fixed-step'});
  }
}
const events=fs.openSync(path.join(directory,'events.jsonl'),'w');
function event(line,extra={}){
  fs.writeSync(events,JSON.stringify({step,sim_time_s:step*dt,line,...extra})+'\n');
}
const encoderBridge=new RrfHttpBridge({encoderResolver:async({axes})=>{
  await advance(0,true);
  controller.handlePayload({type:'encoder_request',requestId:reads++,axes});
  return response;
}});
const angles={A:0,B:0,C:0,D:0};
const gcodeFile=path.resolve('RRF/run/vsd/gcodes/research-51aa157a.gcode');
const canFile=path.resolve(directory,'motion.can');
const captureDir=path.resolve(directory,'motion-profiles');
fs.mkdirSync(captureDir,{recursive:true});
async function send(line){
  event(line);
  if(line.startsWith('M569.3')) return encoderBridge._handleEncoderQuery(line);
  if(line==='M666')return {reply:'M666 A1 Q0 R30:30:30:30 U1:1:1:1 O1:1:1:1 L20:20:20:20 H20:20:20:20 J200:200:200:200',motion:[]};
  if(line==='M669')return {reply:'A:0.00, -1900.00, -280.00\nB:1645.45, 950.00, -280.00\nC:-1645.45, 950.00, -280.00\nD:0.00, 0.00, 1900.00',motion:[]};
  if(line==='M92')return {reply:'M92 E95.922',motion:[]};
  if(line.startsWith('M666 '))return {reply:'',motion:[]};
  if(line.startsWith('M569.4')){
    const ids=line.match(/P([0-9.:]+)/)[1].split(':');
    const force=line.match(/T([-0-9.eE+:]+)/)[1].split(':').map(Number);
    for(let i=0;i<ids.length;i++){
      const axis='ABCD'[Number(ids[i].split('.')[0])-40],f=force[i]??force[0];
      remote.addCommand({type:f===0?'SetPositionMode':'SetTorqueMode',axis,torqueNm:-f*.03});
    }
    torqueCommands++;return {reply:'',motion:[]};
  }
  if(line.startsWith('G1 H2')){
    fs.writeFileSync(gcodeFile,line+'\nM400\n');
    const t=performance.now();
    const result=spawnSync('RRF/build/rrf_simulator',['--vsd','RRF/run/vsd','-c','sys/config_hp3.g',
      '--gcode',gcodeFile,'--can-log',canFile],{encoding:'utf8'});
    planningWallMs+=performance.now()-t;
    if(result.status!==0)throw new Error(result.stderr || result.stdout);
    const capture=fs.readFileSync(canFile,'utf8');
    const profileFile=path.join(captureDir,createHash('sha256').update(line).digest('hex')+'.json');
    if(!fs.existsSync(profileFile))fs.writeFileSync(profileFile,JSON.stringify({line,capture,stdout:result.stdout}));
    const before={...angles};
    const planner=new RrfHttpBridge({remoteSpoolSystem:{addCommand(cmd){
      if(cmd.type==='Move'){
        for(const axis of Object.keys(before)){
          if(Number.isFinite(cmd[axis])){cmd[axis]+=before[axis];angles[axis]=cmd[axis];}
        }
      }
      remote.addCommand(cmd);
    }}});
    const parsed=planner._parseResponse('---MOTION---\n'+capture);
    planner._processMotion(parsed.motion);moves++;
    return parsed;
  }
  throw new Error('Unsupported G-code '+line);
}
send.simulationClock={now:()=>step*dt*1000,sleep:ms=>advance(ms),refresh:async()=>{}};
const cfg=path.resolve(directory,'configs.txt');
fs.writeFileSync(cfg,'[0,3] 1 2\n[1,3] 0 2\n[2,3] 0 1\n');
let status='complete';
try {
  for(const [i,targets] of ['516.922,0','387.692,-100'].entries()){
    const dataset=await collectSweepData(send,{machineType:'hangprinter_4',motorIds:['40.0','41.0','42.0','43.0'],
      speedup:1,args:{sweepConfigFile:cfg,
      fixedTargets:targets,sweepPoints:10,forceLow:.011363575731199994,
      forceMid:.22727151462399986,forceMax:3.354048140226428,noAutoTuneForce:true,
      returnToOrigin:true,outputFile:path.resolve(directory,'sweeps-'+i+'.json')},
      configureSimulator:async()=>{}});
    console.log('batch collected',i,step*dt,!!dataset);
  }
} catch(error){
  status='failed';manifest.error=error.stack;console.error(error);process.exitCode=1;
} finally {
  if(fs.existsSync(gcodeFile))fs.unlinkSync(gcodeFile);
  fs.closeSync(events);
  Object.assign(manifest,{status,steps:step,simulated_s:step*dt,wall_s:(performance.now()-started)/1000,
    moves,encoder_reads:reads,torque_commands:torqueCommands,rrf_planning_s:planningWallMs/1000});
  fs.writeFileSync(path.join(directory,'manifest.json'),JSON.stringify(manifest,null,2));
  console.log(JSON.stringify(manifest));
}
