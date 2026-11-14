// Direct-motor measurement collector on the production headless world (no firmware).
// plan: [{pos:{A:mm,...}, torque:{B:N}, settleS}]  -> per item: encoder mm per axis, effector position, tensions
import fs from 'node:fs';
const R='/home/torbjorn/repos/hp-sim5/';
const { createHeadlessWorld } = await import(R+'hp-sim-3d/app/headlessWorld.js');
const ecs = await import(R+'src/js/cable_joints_3d/ecs.js');
const { CableJointComponent } = await import(R+'src/js/cable_joints_3d/cable_joints_core.js');
const { StepperMotorComponent } = await import(R+'hp-sim-3d/app/hangprinter_stepper_motor.js');
const log=console.log; console.log=console.warn=(...a)=>console.error(...a);
const [planFile,outFile,iters='20',scene='public/usd_scenes/hp3.usda']=process.argv.slice(2);
const { world, remote, dt } = createHeadlessWorld(fs.readFileSync(R+scene,'utf8'), { lineLayeringEnabled:false, closedLoopMotorsEnabled:true, solverIterations:Number(iters) });
const names=new Map(world.query([ecs.SceneEntityInfoComponent]).map(id=>[world.getComponent(id,ecs.SceneEntityInfoComponent).name,id]));
const AX=['A','B','C','D'], RAD=0.03;
const ent=a=>[].concat(remote.axisToEntity[a])[0];
const enc=()=>AX.map(a=>world.getComponent(ent(a),ecs.EncoderComponent).angle);
const eff=()=>{const p=world.getComponent(names.get('Effector'),ecs.PositionComponent).pos; return [p.x,p.y,p.z];};
const groups={A:['JointAL_1','JointAR_1'],B:['JointBL_1','JointBR_1'],C:['JointCL_1','JointCR_1'],D:['JointD0_1','JointD1_1','JointD2_1']};
const tens=()=>AX.map(a=>groups[a].reduce((s,n)=>s+world.getComponent(names.get(n),CableJointComponent).constraintForceMagnitude,0));
const cmd=c=>remote._processCommand(world,c,{recordHistory:false,emitEvents:false});
const step=n=>{for(let i=0;i<n;i++) world.update(dt);};
step(2500);                                   // settle the initial drop
const enc0=enc(), eff0=eff();
const st=a=>world.getComponent(ent(a),StepperMotorComponent);
const cmd0=AX.map(a=>st(a).commandedAngle);
if(planFile==='probe'){
  log(JSON.stringify({enc0,eff0,cmd0,T:tens(),names:[...names.keys()].slice(0,60)}));
  cmd({type:'Move',A:cmd0[0]+1}); step(1500); log(JSON.stringify({afterMoveA1:{enc:enc(),eff:eff(),T:tens(),cmd:AX.map(a=>st(a).commandedAngle)}}));
  cmd({type:'SetTorqueMode',axis:'B',torqueNm:-0.03}); step(3000); log(JSON.stringify({afterTorqueB:{enc:enc(),eff:eff(),T:tens()}}));
  process.exit(0);
}
const plan=JSON.parse(fs.readFileSync(planFile,'utf8')), out=[];
const SPEED=1.0;                                  // rad/s on the spool = 30 mm/s
let mode={A:'pos',B:'pos',C:'pos',D:'pos'};
for(const [k,item] of plan.entries()){
  const tq=Object.keys(item.torque)[0];
  for(const a of AX){                            // lines leaving torque mode hold where they are
    if(mode[a]==='tq' && a!==tq){ const e=enc()[AX.indexOf(a)]; cmd({type:'SetPositionMode',axis:a}); cmd({type:'Move',[a]:e}); mode[a]='pos'; }
  }
  cmd({type:'SetTorqueMode',axis:tq,torqueNm:-item.torque[tq]*RAD}); mode[tq]='tq';
  const start=Object.fromEntries(AX.filter(a=>a!==tq).map(a=>[a,st(a).commandedAngle]));
  const goal=Object.fromEntries(AX.filter(a=>a!==tq).map(a=>[a,item.pos[a]/1000/RAD]));
  const n=Math.max(1,Math.ceil(Math.max(...Object.keys(goal).map(a=>Math.abs(goal[a]-start[a])))/SPEED/dt));
  for(let i=1;i<=n;i++){ const c={type:'Move'}; for(const a in goal) c[a]=start[a]+(goal[a]-start[a])*i/n; cmd(c); step(1); }
  let quiet=0, t=0, last=enc();                  // settle: every encoder still within 2e-6 rad over 0.5 s windows, twice
  while(quiet<2 && t<60){ step(250); t+=250*dt; const e=enc(); quiet=Math.max(...e.map((v,i)=>Math.abs(v-last[i])))<2e-6?quiet+1:0; last=e; }
  const e=enc(), p=eff();
  out.push({k,torqueAxis:tq,forceCmd:item.torque[tq],enc_mm:e.map((v,i)=>(v-enc0[i])*RAD*1000),pos_mm:p.map((v,i)=>(v-eff0[i])*1000),T:tens(),settleS:t,
    cmd_mm:AX.map(a=>st(a).commandedAngle*RAD*1000)});
  if(k%10===0) console.error('pose',k,'/',plan.length,'settle',t.toFixed(1));
}
fs.writeFileSync(outFile,JSON.stringify({eff0,enc0,iters:Number(iters),points:out}));
