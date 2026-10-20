import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { World, PositionComponent, OrientationComponent, PrevFinalPosComponent, MassComponent,
  RigidBodyComponent, RigidBodyMemberComponent, AngularVelocityComponent, MomentOfInertiaComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CablePathComponent, PinholeGateComponent, PBDCableConstraintSolver } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import { WasmCableSolver } from '../../src/js/cable_joints_3d/wasm_cable_solver.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { commands } from './hp4_logo_commands.mjs';
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));

for (const rust of [false, true]) for (const asymmetric of [false,true]) test(`two ports apply one combined carrier Jacobian and wrench: ${rust ? 'Rust kernel' : 'JS'}, asymmetric=${asymmetric}`, () => {
  const w = new World(); w.setResource('dt', 1);
  const body = w.createEntity();
  w.addComponent(body,new PositionComponent()); w.addComponent(body,new PrevFinalPosComponent());
  w.addComponent(body,new MassComponent(1)); w.addComponent(body,new OrientationComponent());
  w.addComponent(body,new MomentOfInertiaComponent([[1,0,0],[0,1,0],[0,0,1]]));
  const ys = [-1,asymmetric ? 2 : 1];
  const ports = ys.map(y=>{const id=w.createEntity(); w.addComponent(id,new PositionComponent(0,y,0));w.addComponent(id,new OrientationComponent());w.addComponent(id,new RigidBodyMemberComponent(body,new Vector3(0,y,0)));return id;});
  w.addComponent(body,new RigidBodyComponent(ports));
  const anchors = ys.map(y=>{const id=w.createEntity();w.addComponent(id,new PositionComponent(1,y,0));return id;});
  const joints = [[anchors[0],ports[0],ys[0]],[ports[1],anchors[1],ys[1]]].map(([a,b,y])=>{const id=w.createEntity();w.addComponent(id,new CableJointComponent(a,b,.9,new Vector3(a===anchors[0]?1:0,y,0),new Vector3(b===anchors[1]?1:0,y,0)));return id;});
  const path = new CablePathComponent(w,joints,['attachment','pinhole_gate','attachment'],[false,false,false],1,[0,asymmetric ? 3 : 2,0]);
  w.addComponent(w.createEntity(),path);
  const backend = rust ? new WasmCableSolver(module) : null;
  try {
    new PBDCableConstraintSolver(backend).update(w,1);
    // C=.2, summed gradient=2, effective inverse mass=4, compliance=1.
    const tension = .2 / (asymmetric ? 6 : 5);
    assert(Math.abs(w.getComponent(body,PositionComponent).pos.x-2*tension)<1e-12);
    const q=w.getComponent(body,OrientationComponent).quaternion;
    assert(Math.abs(q.z-Math.sin((asymmetric ? -tension : 0)/2))<1e-12, 'separate port lever arms preserve torque');
    for(const id of joints) assert(Math.abs(w.getComponent(id,CableJointComponent).constraintForceMagnitude-tension)<1e-12);
    assert.equal(w.getComponent(ports[0],PinholeGateComponent).exitEntity,ports[1]);
    assert.equal(path.stored[1],asymmetric ? 3 : 2);
  } finally {if (backend?.capacity) backend.exports.free_buffer(backend.pointer, backend.capacity);
    if (backend) backend.exports.free_buffer(backend.attachmentPointer, 46);}
});

const source=fs.readFileSync(new URL('../../public/usd_scenes/hp4_pinhole_gate.usda',import.meta.url),'utf8');
const trajectories=[];
for(const [name,backend] of [['native',undefined],['Wasm',()=>new WasmPhysicsEngine(module)],['JS',()=>null]]) test(`HP4 gate short production motion: ${name}`,()=>{
  const {world,remote,dt}=createHeadlessWorld(source,{cableSolverBackend:backend?.()});
  try {
    assert.equal(world.query([PinholeGateComponent]).length,6);
    for(const id of world.query([PinholeGateComponent])) assert.equal(world.getComponent(id,PinholeGateComponent).visualRoute.length,8);
    assert.equal(world.query([CableJointComponent]).length,33);
    assert.equal(world.query([CablePathComponent]).filter(id=>world.getComponent(id,CablePathComponent).linkTypes.includes('pinhole_gate')).length,6);
    remote.commands=commands.slice(0,500);
    const body=world.query([RigidBodyComponent])[0],rows=[];
    for(let i=0;i<500;i++) {
      world.update(dt);
      const p=world.getComponent(body,PositionComponent).pos;
      const omega=world.getComponent(body,AngularVelocityComponent).omega.length();
      assert([p.x,p.y,p.z,omega].every(Number.isFinite));
      assert(Math.abs(p.z-.0903)<.005 && omega<.1,`${name} unstable at ${i}`);
      if(i%50===0) rows.push([p.x,p.y,p.z]);
    }
    assert.equal(remote.getQueueLength(),0);
    assert.equal(getMachineMotorDiagnostics(world).totalMissedSteps,0);
    for(const id of world.query([CablePathComponent])) {
      const p=world.getComponent(id,CablePathComponent);
      if(!p.linkTypes.includes('pinhole_gate'))continue;
      const forces=p.jointEntities.map(id=>world.getComponent(id,CableJointComponent).constraintForceMagnitude);
      assert(forces.every(f=>Math.abs(f-forces[0])<1e-12),'one common gate tension');
    }
    trajectories.push({name,rows});
    const first=trajectories[0].rows;
    for(let i=0;i<rows.length;i++)for(let j=0;j<3;j++)assert(Math.abs(rows[i][j]-first[i][j])<1e-6,`${name} disagrees with native`);
  } finally {world.getResource('cableSolverBackend')?.dispose();}
});
