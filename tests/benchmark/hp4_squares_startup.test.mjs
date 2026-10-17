import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { StepperMotorComponent } from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { SpoolStateComponent } from '../../hp-sim-3d/app/hangprinter_spools.js';
import { EncoderComponent } from '../../src/js/cable_joints_3d/ecs.js';
const root=new URL('../../',import.meta.url);
const fixture=JSON.parse(fs.readFileSync(new URL('../fixtures/hp4_squares_startup.json',import.meta.url)));
const module=new WebAssembly.Module(fs.readFileSync(new URL('public/wasm/cable_solver.wasm',root)));
for(const key of ['source','config']) assert.equal(createHash('sha256').update(fs.readFileSync(new URL(fixture[key],root))).digest('hex'),fixture[key+'Sha256']);
for(const substeps of [4,8,16]) test(`bundled squares: finer substeps resolve B pull-out (${substeps})`,()=>{
  const backend=new WasmPhysicsEngine(module);
  try {
    const {world,remote,dt}=createHeadlessWorld(fs.readFileSync(new URL('public/usd_scenes/hp4_rigid_body.usda',root),'utf8'),{physicsSubsteps:substeps,cableSolverBackend:backend});
    assert.equal(dt,fixture.dt);
    remote.commands=fixture.commands.slice();
    for(const _command of fixture.commands) world.update(dt);
    assert.equal(remote.getQueueLength(),0);
    const id=world.query([StepperMotorComponent]).find(id=>world.getComponent(id,SpoolStateComponent).axis==='B');
    const phaseError=world.getComponent(id,EncoderComponent).angle-world.getComponent(id,StepperMotorComponent).commandedAngle;
    // A 32/64-substep reference settles near a two-period lag (0.255 rad).
    // Testing zero losses here would preserve the under-resolved 1-step result.
    assert(Math.abs(phaseError-2*(2*Math.PI/50))<.02);
    assert.equal(getMachineMotorDiagnostics(world).motors.find(m=>m.axis==='B').missedSteps,2);
  } finally {backend.dispose();}
});
