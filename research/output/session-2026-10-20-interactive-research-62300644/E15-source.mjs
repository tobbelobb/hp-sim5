import fs from 'node:fs';
import * as E from '../../../../src/js/cable_joints_3d/ecs.js';
import {CableJointComponent as Joint,CablePathComponent as Path,CableLinkComponent as Link} from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import {SpoolStateComponent} from '../../../../hp-sim-3d/app/hangprinter_spools.js';
import {CableFrictionSystem} from '../../../../src/js/cable_joints_3d/cable_friction_system.js';
import {registerSimulationSystems} from '../../../../hp-sim-3d/app/simulationSystems.js';
import Vector3 from '../../../../src/js/cable_joints_3d/vector3.js';

// One inertial wheel, immobile axle, two horizontal taut legs ending at fixed
// anchors. Analytic equilibrium: |theta|=.001/(2r), T=K*.001/2.
const rows=[];
for(const redistribution of [true,false]) {
 const w=new E.World(),r=.0065,I=5e-7,k=20000,dt=.0005;
 w.setResource('dt',dt);w.setResource('enableLayering',false);
 const wheel=w.createEntity();
 for(const c of [new E.PositionComponent(),new E.PrevFinalPosComponent(),new E.MassComponent(0),new E.VelocityComponent(),new E.OrientationComponent(),new E.PrevFinalOrientationComponent(),new E.AngularVelocityComponent(),new E.MomentOfInertiaComponent([[I,0,0],[0,I,0],[0,0,I]]),new E.RadiusComponent(r),new E.EncoderComponent(),new Link(0,0,0,null,null,new Vector3(0,0,1)),new SpoolStateComponent()]) w.addComponent(wheel,c);
 const anchors=[r,-r].map(y=>{const a=w.createEntity();w.addComponent(a,new E.PositionComponent(-1,y,0));w.addComponent(a,new E.MassComponent(0));return a;});
 const js=[new Joint(anchors[0],wheel,.999,new Vector3(-1,r,0),new Vector3(0,r,0)),new Joint(wheel,anchors[1],1,new Vector3(0,-r,0),new Vector3(-1,-r,0))];
 const ids=js.map(j=>{const id=w.createEntity();w.addComponent(id,j);return id;});
 const path=new Path(w,ids,['attachment','rolling','attachment'],[true,true,true],k,null,0,1,8);
 w.addComponent(w.createEntity(),path);
 registerSimulationSystems(w,{cableSolverBackend:null});
 if(!redistribution) w.systems.find(s=>s instanceof CableFrictionSystem).update=()=>{};
 const samples=[];
 for(let step=0;step<=2000;step++) {
  if(step%10===0) samples.push({t:step*dt,angle:w.getComponent(wheel,E.EncoderComponent).angle,omega:w.getComponent(wheel,E.AngularVelocityComponent).omega.z,rests:js.map(j=>j.restLength),tensions:js.map(j=>Math.max(0,k*(j.attachmentPointA_world.distanceTo(j.attachmentPointB_world)-j.restLength)))});
  if(step<2000)w.update(dt);
 }
 rows.push({redistribution,r,I,k,dt,expected_abs_angle:.001/(2*r),expected_tension:k*.001/2,samples});
}
fs.writeFileSync(new URL('E15-pulley.json',import.meta.url),JSON.stringify(rows,null,2));
console.log(JSON.stringify(rows.map(r=>({redistribution:r.redistribution,expected_abs_angle:r.expected_abs_angle,final:r.samples.at(-1)}))));
