import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as ecs from '../../src/js/cable_joints_3d/ecs.js';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import {CableJointComponent,CableLinkComponent,CablePathComponent} from '../../src/js/cable_joints_3d/cable_joints_core.js';
import {StepperMotorComponent} from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import {SpoolStateComponent} from '../../hp-sim-3d/app/hangprinter_spools.js';
import {registerSimulationSystems} from '../../hp-sim-3d/app/simulationSystems.js';
import {WasmCableSolver} from '../../src/js/cable_joints_3d/wasm_cable_solver.js';
import {WasmPhysicsEngine} from '../../src/js/cable_joints_3d/wasm_physics_engine.js';

const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm',import.meta.url)));
// tau_motor = H*sin(p*theta), tau_cable = r*k*(extension-r*theta).
// This physical equilibrium contains no timestep or solver-iteration factor.
let low=0, high=.02;
for(let i=0;i<60;i++) {
  const middle=(low+high)/2;
  if(.5*Math.sin(50*middle)<.2-9*middle) low=middle; else high=middle;
}
const expectedAngle=(low+high)/2;
for(const kind of ['static torque','cable damping']) for(const engine of ['JavaScript','Wasm kernel','Wasm physics']) for(const substeps of [1,2,4,8,16]) {
  test(`${engine}: open-loop ${kind} at ${substeps} substeps`,()=>{
    const backend=engine==='Wasm physics'?new WasmPhysicsEngine(module):engine==='Wasm kernel'?new WasmCableSolver(module):null;
    const world=new ecs.World(),dt=.002/substeps,damper=kind==='cable damping';
    world.setResource('dt',dt);world.setResource('gravity',new Vector3());
    const anchor=world.createEntity(),spool=world.createEntity(),joint=world.createEntity(),path=world.createEntity();
    world.addComponent(anchor,new ecs.PositionComponent(-1,0,0));world.addComponent(anchor,new ecs.MassComponent(-1));
    for(const Type of [ecs.PositionComponent,ecs.OrientationComponent,ecs.PrevFinalOrientationComponent,ecs.PrevFinalPosComponent,ecs.AngularVelocityComponent,ecs.EncoderComponent])world.addComponent(spool,new Type());
    world.addComponent(spool,new ecs.MassComponent(0));world.addComponent(spool,new ecs.MomentOfInertiaComponent(5e-5));
    world.addComponent(spool,new ecs.RadiusComponent(.03));world.addComponent(spool,new SpoolStateComponent('A'));
    world.addComponent(spool,new StepperMotorComponent(0,0,damper?0:.5,50,damper?0:.011));
    world.addComponent(spool,new CableLinkComponent(0,0,0,null,null,new Vector3(0,0,1)));
    world.addComponent(joint,CableJointComponent.fromWorld(anchor,spool,Math.sqrt(1-.03**2)-(.2/.03)/10000,new Vector3(-1,0,0),new Vector3(-.0009,Math.sqrt(.0009-.0009**2),0)));
    world.addComponent(path,new CablePathComponent(world,[joint],['attachment','rolling'],[true,true],damper?0:10000,[0,0],0,damper?5:0,1));
    registerSimulationSystems(world,{cableSolverBackend:backend});
    try {
      if(damper) world.getComponent(spool,ecs.AngularVelocityComponent).omega.z=-1;
      for(let i=0;i<(damper?1:500*substeps);i++)world.update(dt);
      const angle=world.getComponent(spool,ecs.EncoderComponent).angle;
      const constraint=world.getComponent(joint,CableJointComponent);
      const cableTorque=Math.abs(constraint.attachmentPointB_world.clone().cross(constraint.constraintForce).z);
      if(damper) {
        // Backward Euler for I*domega/dt = -damping*r²*omega.
        const expected=-1/(1+5*.03**2*dt/5e-5);
        assert(Math.abs(world.getComponent(spool,ecs.AngularVelocityComponent).omega.z-expected)<1e-12);
        assert(Math.abs(cableTorque-5*.03**2*Math.abs(expected))<1e-12,'count rotor displacement once');
      } else {
        assert(Math.abs(angle-expectedAngle)<1e-10,`${angle} != ${expectedAngle}`);
        assert(Math.abs(.5*Math.sin(50*angle)-cableTorque)<1e-9,'motor must balance the full cable torque');
      }
    } finally {backend?.dispose?.();}
  });
}
