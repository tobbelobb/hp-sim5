import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
const root=process.cwd();
const {runFixture}=await import(pathToFileURL(`${root}/tests/parity3d/oracle.mjs`));
const {NativePhysicsEngine}=await import(pathToFileURL(`${root}/src/js/cable_joints_3d/native_physics_engine.js`));
const session=`${root}/research/sessions/attempt-0b9a865f76514b04930528d43f2c31ea`;
const fixture=JSON.parse(fs.readFileSync(`${root}/tests/fixtures/python_3d_parity/open_loop_static_load.json`));
let low=0,high=.02;
for(let i=0;i<60;i++){const m=(low+high)/2;if(.5*Math.sin(50*m)<.2-9*m) low=m;else high=m;}
const expected=(low+high)/2, results=[];
for(const label of ['old','current']){
 const input=structuredClone(fixture);input.resources.dt=.002;input.steps=Array.from({length:500},()=>({dt:.002}));
 const backend=new NativePhysicsEngine({addonPath:`${session}/evidence/native/${label}.node`});
 if(label==='old'){const original=backend.transport.step;backend.transport.step=dt=>{backend.optionsRegion.data[12]=1;return original(dt);};}
 try{const state=runFixture(input,{cableSolverBackend:backend}).snapshots.at(-1).entities;
 const angle=state.spool.EncoderComponent.angle;
 results.push({label,steps:500,dt_s:.002,simulated_s:1,expected_angle_rad:expected,angle_rad:angle,absolute_angle_error_rad:Math.abs(angle-expected),motor_torque_nm:.5*Math.sin(50*angle),cable_torque_nm:.03*state.joint.CableJointComponent.constraintForceMagnitude});
 }finally{backend.dispose();}
}
fs.writeFileSync(`${session}/evidence/torque-check.json`,JSON.stringify(results,null,2)+'\n');
console.log(JSON.stringify(results));
