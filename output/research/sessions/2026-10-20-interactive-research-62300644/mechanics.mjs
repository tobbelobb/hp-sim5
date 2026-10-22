import fs from 'node:fs';
import {createHash} from 'node:crypto';
import {createHeadlessWorld} from '../../../../hp-sim-3d/app/headlessWorld.js';
import * as E from '../../../../src/js/cable_joints_3d/ecs.js';
import {CableJointComponent as Joint, CablePathComponent as Path, PBDCableConstraintSolver} from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import {CableFrictionSystem} from '../../../../src/js/cable_joints_3d/cable_friction_system.js';
import {TorqueModeSystem} from '../../../../hp-sim-3d/app/torqueModeSystem.js';
import {StepperMotorComponent as Motor} from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import {SpoolStateComponent} from '../../../../hp-sim-3d/app/hangprinter_spools.js';
import Vector3 from '../../../../src/js/cable_joints_3d/vector3.js';

const root = new URL('./', import.meta.url);
const hash = x => createHash('sha256').update(x).digest('hex');
const write = (name, x) => fs.writeFileSync(new URL(name, root), JSON.stringify(x, null, 2));
const xyz = v => [v.x,v.y,v.z];

// Isolated constitutive operation: frozen geometry, frozen wheel angle.
if (process.argv[2] === 'fixture') {
  const w = new E.World(), ids = [w.createEntity(),w.createEntity(),w.createEntity()];
  w.addComponent(ids[1],new SpoolStateComponent());
  w.addComponent(ids[1],new E.RadiusComponent(.0065));
  w.addComponent(ids[1],new E.EncoderComponent());
  const joints = [
    new Joint(ids[0],ids[1],.999,new Vector3(-1,0,0),new Vector3()),
    new Joint(ids[1],ids[2],1,new Vector3(),new Vector3(1,0,0)),
  ];
  const js = joints.map(j=>{const id=w.createEntity();w.addComponent(id,j);return id;});
  // Avoid tangent construction; the fixture only invokes the redistribution law.
  const path = Object.assign(Object.create(Path.prototype),{jointEntities:js,linkTypes:['attachment','rolling','attachment'],stored:[0,0,0],cableHalfWidth:0});
  w.addComponent(w.createEntity(),path);
  const before = joints.map(j=>j.restLength);
  new CableFrictionSystem().update(w,.002);
  write('E01-frozen-wheel.json',{before,after:joints.map(j=>j.restLength),encoder:w.getComponent(ids[1],E.EncoderComponent).angle,
    apparentCableTransport_m:joints[0].restLength-before[0],dt:.002,operation:'CableFrictionSystem only; no mechanics step'});
  process.exit(0);
}

const [id,variant,backend,subArg,mode,secondsArg='30'] = process.argv.slice(2);
const extraNoSlip=process.argv.includes('--no-slip');
if (!id || !['stock','baseline-friction','no-rolling-slip','no-passive-transfer','accumulated-load','accumulated-motor','motor','gate','gate-matched'].includes(variant)) throw Error('Unknown experiment');
let scene=fs.readFileSync('public/usd_scenes/hp4_rigid_body.usda','utf8');
if (variant.startsWith('gate')) {
  for (const branch of ['AL','AR','BL','BR','CL','CR']) {
    scene=scene.replace(new RegExp(`        def CableJoint "Joint${branch}_pinhole_to_wheel"[\\s\\S]*?(?=        def CableJoint "Joint${branch}_pinhole_to_anchor")`),
      `        def CableJoint "Joint${branch}_gate"\n        {\n            custom rel physics:body0 = </World/HangprinterScene/Pinhole${branch}_top>\n            custom rel physics:body1 = </World/HangprinterScene/Pinhole${branch}_bottom>\n        }\n\n`);
    const re=new RegExp(`        def Xform "CablePath${branch}"[\\s\\S]*?\\n        }`);
    scene=scene.replace(re,block=>block
      .replace(/custom bool\[\] cablePath:clockwise = .*/, 'custom bool[] cablePath:clockwise = [1, 1, 1, 1, 1]')
      .replace(/custom rel cablePath:joints = .*/, `custom rel cablePath:joints = [</World/HangprinterScene/Joint${branch}_0>, </World/HangprinterScene/Joint${branch}_anchor_to_pinhole>, </World/HangprinterScene/Joint${branch}_gate>, </World/HangprinterScene/Joint${branch}_pinhole_to_anchor>]`)
      .replace(/custom token\[\] cablePath:linkTypes = .*/, 'custom token[] cablePath:linkTypes = ["hybrid", "pinhole", "pinhole", "pinhole", "attachment"]')
      .replace(/custom double\[\] cablePath:stored = .*/, 'custom double[] cablePath:stored = [2.0, 0, 0, 0, 0]')
      .replace(/custom token\[\] cablePath:storedMode = .*/, 'custom token[] cablePath:storedMode = ["manual", "manual", "manual", "manual", "manual"]')
      .replace('stiffness = 20000.0',`stiffness = ${variant==='gate-matched'?20000*4/6:20000}`)
      .replace('damping = 5.0',`damping = ${variant==='gate-matched'?5*4/6:5}`));
  }
}
fs.writeFileSync(new URL(`${id}-input.usda`,root),scene);
const substeps=Number(subArg), seconds=Number(secondsArg);
const {world:w,remote,dt,bakedScene}=createHeadlessWorld(scene,{lineLayeringEnabled:false,closedLoopMotorsEnabled:true,physicsSubsteps:substeps,...(backend==='js'?{cableSolverBackend:null}:{})});
fs.writeFileSync(new URL(`${id}-baked.usda`,root),bakedScene);
if (variant==='baseline-friction') {
  if (backend!=='js')throw Error('Variant needs JS');
  const sourceURL=new URL('../../../../src/js/cable_joints_3d/cable_friction_system.js',root);
  const source=fs.readFileSync(new URL('baseline-source/src/js/cable_joints_3d/cable_friction_system.js',root),'utf8')
    .replace(/from '([^']+)'/g,(_,p)=>`from '${new URL(p,sourceURL).href}'`);
  fs.writeFileSync(new URL('baseline-friction.mjs',root),source);
  const {CableFrictionSystem:Baseline}=await import('./baseline-friction.mjs');
  w.systems.find(s=>s instanceof CableFrictionSystem).update=new Baseline().update;
}
if (extraNoSlip || ['no-rolling-slip','no-passive-transfer'].includes(variant)) {
  if (backend!=='js') throw Error('Variant needs JS');
  const friction=w.systems.find(s=>s instanceof CableFrictionSystem), original=friction.update.bind(friction);
  friction.update=(world,dt)=>{
    const paths=world.query([Path]).map(id=>world.getComponent(id,Path));
    const old=paths.map(p=>p.linkTypes);
    paths.forEach(p=>p.linkTypes=p.linkTypes.map(t=>t==='rolling'?'attachment':t));
    try {original(world,dt);} finally {paths.forEach((p,i)=>p.linkTypes=old[i]);}
  };
}
if (['no-passive-transfer','accumulated-load','accumulated-motor'].includes(variant)) {
  const sourceURL=new URL('../../../../src/js/cable_joints_3d/cable_joints_core.js',root);
  let source=fs.readFileSync(sourceURL,'utf8');
  for(const name of ['CableLinkComponent','CableJointComponent','CablePathComponent']) {
    const start=source.indexOf(`export class ${name} {`), open=source.indexOf('{',start);
    let depth=1,end=open+1;
    for(;depth;end++) {if(source[end]==='{')depth++;if(source[end]==='}')depth--;}
    source=source.slice(0,start)+source.slice(end);
  }
  source=`import {CableLinkComponent,CableJointComponent,CablePathComponent} from '${sourceURL.href}';\n`+source;
  if(variant==='no-passive-transfer') source=source.replace("return linkType === 'rolling' || _isHybrid(linkType);",'return _isHybrid(linkType);');
  if(variant.startsWith('accumulated-')) {
    source=source.replace('const recordTorqueModeCableLoad = (path, memberSpin, loadGradSpin, lambda, invDtSq) => {',
      'const recordTorqueModeCableLoad = (path, memberSpin, loadGradSpin, lambda, invDtSq, recordCoefficients) => {');
    source=source.replace('if (memberSpin.recordTorqueLoadImplicitCoeffs) {','if (memberSpin.recordTorqueLoadImplicitCoeffs && recordCoefficients) {');
    const old=`if (solveIteration === 0) {\n        recordTorqueModeCableLoad(path, memberSpinA, loadGradSpinA, lambda, invDtSq);\n        recordTorqueModeCableLoad(path, memberSpinB, loadGradSpinB, lambda, invDtSq);\n      }`;
    if(!source.includes(old))throw Error('Missing torque-load block');
    source=source.replace(old,`recordTorqueModeCableLoad(path, memberSpinA, loadGradSpinA, lambda, invDtSq, solveIteration === 0);\n      recordTorqueModeCableLoad(path, memberSpinB, loadGradSpinB, lambda, invDtSq, solveIteration === 0);`);
  }
  source=source.replace(/from '([^']+)'/g,(_,p)=>`from '${new URL(p,sourceURL).href}'`);
  fs.writeFileSync(new URL('candidate-core.mjs',root),source);
  const {PBDCableConstraintSolver:Candidate}=await import('./candidate-core.mjs');
  const candidate=new Candidate();
  w.systems.find(s=>s instanceof PBDCableConstraintSolver).update=candidate.update.bind(candidate);
}
if (['motor','accumulated-motor'].includes(variant)) {
  if (backend!=='js') throw Error('Variant needs JS');
  // Reproduce the previous candidate without editing production code.
  const sourceURL=new URL('../../../../hp-sim-3d/app/torqueModeSystem.js',root);
  let source=fs.readFileSync(sourceURL,'utf8').replace('driveTorque + springLoadTorque)', 'driveTorque + springLoadTorque + dt * cableLoadStiffness * omegaAlongAxis)');
  source=source.replace(/from '([^']+)'/g,(_,p)=>`from '${new URL(p,sourceURL).href}'`);
  fs.writeFileSync(new URL('candidate-torque.mjs',root),source);
  const {TorqueModeSystem:Candidate}=await import('./candidate-torque.mjs');
  w.systems.find(s=>s instanceof TorqueModeSystem).update=new Candidate().update;
}
const body=w.query([E.RigidBodyComponent])[0];
const name=id=>w.getComponent(id,E.SceneEntityInfoComponent)?.name??String(id);
const wheels=w.query([E.SceneEntityInfoComponent]).filter(id=>name(id).startsWith('Wheel'));
const paths=w.query([Path]);
const sample=step=>({t:step*dt,xyz:xyz(w.getComponent(body,E.PositionComponent).pos),velocity:xyz(w.getComponent(body,E.VelocityComponent).vel),omega:xyz(w.getComponent(body,E.AngularVelocityComponent).omega),
  wheels:wheels.map(id=>({name:name(id),angle:w.getComponent(id,E.EncoderComponent)?.angle,omega:xyz(w.getComponent(id,E.AngularVelocityComponent).omega)})),
  motors:[...'ABCD'].map(axis=>{const id=remote.axisToEntity[axis][0];return {axis,angle:w.getComponent(id,E.EncoderComponent).angle,K:w.getResource('torqueModeCableLoadStiffnesses')?.get(id)??0,load:w.getResource('torqueModeCableLoadTorques')?.get(id)??0};}),
  paths:paths.map(id=>{const p=w.getComponent(id,Path);return {name:name(id),stored:p.stored,total:p.totalRestLength,joints:p.jointEntities.map(id=>{const j=w.getComponent(id,Joint);return {name:name(id),a:name(j.entityA),b:name(j.entityB),rest:j.restLength,length:j.attachmentPointA_world.distanceTo(j.attachmentPointB_world),force:j.constraintForceMagnitude,transferred:j.transferredConstraintForceMagnitude,pa:xyz(j.attachmentPointA_world),pb:xyz(j.attachmentPointB_world)};})};})});
const rows=[],steps=Math.round(seconds/dt),start=performance.now();
try {
  for(let step=0;step<=steps;step++) {
    if(mode==='pull'&&step===1000) for(const [axis,torqueNm] of [['A',-.000334089],['B',-.037733],['C',-.037733]]) remote.addCommand({type:'SetTorqueMode',axis,torqueNm});
    if(step%50===0) rows.push(sample(step));
    if(!Number.isFinite(w.getComponent(body,E.PositionComponent).pos.length()) || w.getComponent(body,E.PositionComponent).pos.length()>10) throw Error('Diverged');
    if(backend==='js'&&step===steps-1) w.setResource('solverTrace',{rows:[],step});
    if(step<steps)w.update(dt);
  }
  const result={id,variant,backend,substeps,mode,dt,steps,seconds,wall_s:(performance.now()-start)/1000,scene_sha256:hash(scene),baked_sha256:hash(bakedScene),engine:w.getResource('physicsEngine'),artifact:w.getResource('physicsArtifact'),mass:w.getComponent(body,E.MassComponent).mass,solverTrace:w.getResource('solverTrace'),entityNames:Object.fromEntries(w.query([E.SceneEntityInfoComponent]).map(id=>[id,name(id)])),rows};
  write(`${id}.json`,result);
  console.log(JSON.stringify({id,variant,backend,substeps,mode,wall_s:result.wall_s,finalXYZ:rows.at(-1).xyz,maxWheelAngle:Math.max(...rows.at(-1).wheels.map(w=>Math.abs(w.angle)))}));
} catch(error) {write(`${id}-failed.json`,{id,error:String(error),rows});throw error;}
finally {w.getResource('cableSolverBackend')?.dispose();}
