import fs from 'node:fs';
import {createHeadlessWorld} from '../../../../hp-sim-3d/app/headlessWorld.js';
import {PositionComponent,RigidBodyComponent,EncoderComponent,CoefficientOfFrictionComponent} from '../../../../src/js/cable_joints_3d/ecs.js';
import {CablePathComponent,CableJointComponent} from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import {SpoolStateComponent} from '../../../../hp-sim-3d/app/hangprinter_spools.js';
const variant=process.argv[2]||'baseline',seconds=Number(process.argv[3]||30);
const {world,remote,dt}=createHeadlessWorld(fs.readFileSync('public/usd_scenes/hp4_rigid_body.usda','utf8'),{lineLayeringEnabled:false,closedLoopMotorsEnabled:true,cableSolverBackend:null,physicsSubsteps:variant==='substeps2'?2:1});
const paths=world.query([CablePathComponent]).map(id=>world.getComponent(id,CablePathComponent));
for(const p of paths)for(let i=1;i<p.linkTypes.length-1;i++){
 const j=world.getComponent(p.jointEntities[i-1],CableJointComponent);
 const mu=world.getComponent(j.entityB,CoefficientOfFrictionComponent)?.mu??0;
 if(mu>0 && !(p.linkTypes[i]==='rolling'&&world.getComponent(j.entityB,SpoolStateComponent)))throw Error('Counterfactual requires frictionless internal links');
}
if(variant==='iterations1')for(const p of paths)p.solverIterations=1;
if(variant==='global'){
 const friction=world.systems.find(s=>s.constructor.name==='CableFrictionSystem');
 friction.update=()=>{for(const p of paths){
  const js=p.jointEntities.map(id=>world.getComponent(id,CableJointComponent));
  const ds=js.map(j=>j.attachmentPointA_world.distanceTo(j.attachmentPointB_world));
  const total=js.reduce((s,j)=>s+j.restLength,0),dist=ds.reduce((a,b)=>a+b,0);
  let rest;
  if(total>=dist)rest=ds.map(d=>total*d/dist);
  else{
   let active=ds.map((d,i)=>i),remaining=total;rest=ds.map(()=>0);
   while(active.length){const extension=(active.reduce((s,i)=>s+ds[i],0)-remaining)/active.length;
    const kept=active.filter(i=>ds[i]>extension);
    if(kept.length===active.length){for(const i of active)rest[i]=ds[i]-extension;break;}
    active=kept;
   }
  }
  js.forEach((j,i)=>j.restLength=rest[i]);
 }};
}
if(variant==='no-implicit-stiffness'){
 const motor=world.systems.find(s=>s.constructor.name==='TorqueModeSystem'),update=motor.update.bind(motor);
 motor.update=(state,dt)=>{const old=state.getResource('torqueModeCableLoadStiffnesses');state.setResource('torqueModeCableLoadStiffnesses',new Map([...old].map(([id])=>[id,1e-30])));try{update(state,dt);}finally{state.setResource('torqueModeCableLoadStiffnesses',old);}};
}
const body=world.query([RigidBodyComponent])[0],rows=[],start=performance.now();
for(let step=0;step<=seconds/dt;step++){
 if(step===1000)for(const [axis,torqueNm] of [['A',-.000334089],['B',-.037733],['C',-.037733]])remote.addCommand({type:'SetTorqueMode',axis,torqueNm});
 if(step%500===0){const p=world.getComponent(body,PositionComponent).pos;
  rows.push({t:step*dt,xyz:[p.x,p.y,p.z],enc:[...'ABCD'].map(axis=>world.getComponent(remote.axisToEntity[axis][0],EncoderComponent).angle*180/Math.PI),loadK:[...'ABCD'].map(axis=>world.getResource('torqueModeCableLoadStiffnesses')?.get(remote.axisToEntity[axis][0])??0)});
  if(!Number.isFinite(p.length())||p.length()>10)break;
 }
 if(step<seconds/dt)world.update(dt);
}
const result={variant,seconds,wall_s:(performance.now()-start)/1000,rows};fs.writeFileSync(new URL(`${variant}.json`,import.meta.url),JSON.stringify(result,null,2));console.log(result.variant,result.wall_s,rows.at(-1));
