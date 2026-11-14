import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
const root=process.cwd();
const {runFixture}=await import(pathToFileURL(`${root}/tests/parity3d/oracle.mjs`));
const {NativePhysicsEngine}=await import(pathToFileURL(`${root}/src/js/cable_joints_3d/native_physics_engine.js`));
const fixture=JSON.parse(fs.readFileSync(`${root}/tests/fixtures/python_3d_parity/open_loop_static_load.json`));
fixture.resources.dt=.002;fixture.steps=Array.from({length:500},()=>({dt:.002}));
const backend=new NativePhysicsEngine();
try{
 const state=runFixture(fixture,{cableSolverBackend:backend}).snapshots.at(-1).entities;
 console.log(JSON.stringify({label:process.argv[2],angle_rad:state.spool.EncoderComponent.angle,motor_torque_nm:.5*Math.sin(50*state.spool.EncoderComponent.angle),cable_torque_nm:.03*state.joint.CableJointComponent.constraintForceMagnitude,artifact:backend.artifactInfo.sha256}));
}finally{backend.dispose();}
