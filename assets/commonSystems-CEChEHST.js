var E=Object.defineProperty;var G=(i,e,m)=>e in i?E(i,e,{enumerable:!0,configurable:!0,writable:!0,value:m}):i[e]=m;var v=(i,e,m)=>G(i,typeof e!="symbol"?e+"":e,m);import{P as b,a as V,O as R,f as w,b as M,G as j,A as O,M as x,e as _,V as A,o as F}from"./cable_joints_core-oiO67cBX.js";function L(i){const e=i.systems.find(a=>a.constructor.name==="RenderSystem"),m=e?e.viewScaleMultiplier:1,p=e?e.viewOffsetX_sim:0,r=e?e.viewOffsetY_sim:0;let o=`
// --- Generated Error Test Case ---
testGeneratedError: {
  // Viewport settings from the time of the error
  viewport: { scale: ${m}, offsetX: ${p}, offsetY: ${r} },
  run: function() {
    const testName = "Generated Error Case";
    const world = new World();
    window.testWorld = world; // Set global world
    world.setResource('dt', ${i.getResource("dt")});
    world.setResource('debugRenderPoints', {});
    world.setResource('simWidth', ${i.getResource("simWidth")});
    world.setResource('simHeight', ${i.getResource("simHeight")});

    // --- Resources ---
`;const s=i.getResource("gravity");s&&(o+=`    world.setResource('gravity', new Vector2(${s.x}, ${s.y}));
`);const l=i.getResource("errorState");l&&(o+=`    world.setResource('errorState', new SimulationErrorStateComponent(${l.hasError}));
`),o+=`
    // --- Entities and Components ---
`;const c=new Map;for(const a of i.entities.keys()){const u=`entity${a}`;c.set(a,u),o+=`    const ${u} = world.createEntity(); // Original ID: ${a}
`}o+=`
`;for(const a of i.entities.keys()){const u=c.get(a),h=i.entities.get(a);if(h){for(const g of h){const t=i.getComponent(a,g);if(!t)continue;let n=`    world.addComponent(${u}, new ${g.name}(`;switch(g.name){case"PositionComponent":case"PrevFinalPosComponent":n+=`${t.pos.x}, ${t.pos.y}`;break;case"VelocityComponent":n+=`${t.vel.x}, ${t.vel.y}`;break;case"RadiusComponent":n+=`${t.radius}`;break;case"MassComponent":n+=`${t.mass}`;break;case"ObstaclePushComponent":n+=`${t.pushVel}`;break;case"ScoreComponent":n+=`${t.value}`;break;case"RestitutionComponent":n+=`${t.restitution}`;break;case"CoefficientOfFrictionComponent":n+=`${t.mu}`;break;case"OrientationComponent":case"PrevFinalOrientationComponent":n+=`${t.angle}`;break;case"AngularVelocityComponent":n+=`${t.angularVelocity}`;break;case"MomentOfInertiaComponent":n+=`${t.inertia}`;break;case"RenderableComponent":n+=`'${t.shape}', '${t.color}'`;break;case"FlipperStateComponent":n+=`${t.length}, ${t.restAngle},  ${t.sign*t.maxRotation}, ${t.angularVelocity}`;break;case"FlipperTipComponent":const P=c.get(t.flipperEntityId);P?n+=P:(console.warn(`Could not find variable name for flipper entity ${t.flipperEntityId} in FlipperTipComponent for entity ${a}`),n=`    // Skipped FlipperTipComponent for ${u} due to missing entity mapping
`);break;case"CableJointComponent":const S=c.get(t.entityA),$=c.get(t.entityB);!S||!$?(console.warn(`Could not find variable names for entities ${t.entityA} or ${t.entityB} in joint ${a}`),n=`    // Skipped CableJointComponent for ${u} due to missing entity mapping
`):t.attachmentPointA_world&&t.attachmentPointB_world?(n+=`${S}, ${$}, ${t.restLength}, `,n+=`new Vector2(${t.attachmentPointA_world.x}, ${t.attachmentPointA_world.y}), `,n+=`new Vector2(${t.attachmentPointB_world.x}, ${t.attachmentPointB_world.y})`):n=`    // Skipped CableJointComponent for ${u}: could not find local attachment point properties (e.g., 'attachmentPointA_world').
`;break;case"PauseStateComponent":n+=t.paused;break;case"SimulationErrorStateComponent":n+=t.hasError;break;case"BorderComponent":const f=t.points.map(C=>`new Vector2(${C.x}, ${C.y})`).join(", ");n+=`[${f}]`;break;case"CablePathComponent":n=`
`;break;case"GravityAffectedComponent":case"CableLinkComponent":case"BallTagComponent":case"FlipperTagComponent":case"ObstacleTagComponent":case"ScoredTagComponent":break;default:console.warn(`Unhandled component type for serialization: ${g.name}`),n=`    // Skipped unknown component ${g.name} for ${u}
`}n.endsWith(`
`)||(n+=`));
`),o+=n}o+=`
`}}for(const a of i.entities.keys()){const u=c.get(a),h=i.entities.get(a);if(h)for(const g of h){const t=i.getComponent(a,g);if(!t)continue;let n="";if(g.name==="CablePathComponent"){n+=`    world.addComponent(${u}, new ${g.name}(`;const P=t.jointEntities.map(S=>c.get(S)).filter(Boolean);if(P.length!==t.jointEntities.length)console.warn(`Could not find variable names for all joints in path ${a}`),n=`    // Skipped CablePathComponent for ${u} due to missing joint mapping
`;else{n+=`world, [${P.join(", ")}], `,n+=`[${t.linkTypes.map(S=>`'${S}'`).join(", ")}], `,n+=`[${t.cw.join(", ")}], `,n+=`${t.spring_constant}, `,n+=`[${t.stored.join(", ")}]`,n+=`));
`,n+=`    const pathComp_${u} = world.getComponent(${u}, CablePathComponent);
`,n+=`    pathComp_${u}.totalRestLength = ${t.totalRestLength};
`,o+=n;continue}}!n.endsWith(`
`)&&n!==""&&(n+=`));
`),o+=n}}return o+=`
`,o+=`
    // --- Systems Registration (Copy from a working test or main file if needed) ---
    // world.registerSystem(new CableAttachmentUpdateSystem());
    // world.registerSystem(new PBDCableConstraintSolver());
    // ... etc ...

    // --- Run the system that caused the error (or the full update) ---
    // const system = new SpecificSystem();
    // system.update(world, world.getResource('dt'));
    // OR
    // world.update(world.getResource('dt')); // If the error happens during the full update cycle

    // --- Assertions (Add assertions here to verify the error or expected state) ---
    // assertTrue(..., testName, "Some condition");

    logTestResult(testName, false, "Generated test case - needs verification and assertions");
    return world; // Return world for rendering
  }
},
`,o}class W{constructor(){v(this,"runInPause",!1)}update(e,m){const p=e.query([R,w]);for(const r of p){const o=e.getComponent(r,R),s=e.getComponent(r,w);s.angle=o.angle}}}class N{constructor(){v(this,"runInPause",!1)}_normalizeAngle(e){for(;e>Math.PI;)e-=2*Math.PI;for(;e<-Math.PI;)e+=2*Math.PI;return e}update(e,m){const p=e.getResource("grabbedBall"),r=e.query([R,O,w,_]);if(!(m<=1e-9))for(const s of r){if(s===p)continue;const l=e.getComponent(s,_);if(l&&l.invInertia<=0)continue;const c=e.getComponent(s,R),a=e.getComponent(s,O),u=e.getComponent(s,w),h=c.angle,g=u.angle;let t=this._normalizeAngle(h-g);a.angularVelocity=t/m}}}class D{constructor(){v(this,"runInPause",!1)}update(e,m){const p=e.getResource("grabbedBall"),r=e.query([b,M,V,x]);if(!(m<=1e-9))for(const s of r){if(s===p)continue;const l=e.getComponent(s,x);if(l&&l.mass<=0)continue;const c=e.getComponent(s,b),a=e.getComponent(s,M),u=e.getComponent(s,V);a.vel.subtractVectors(c.pos,u.pos).scale(1/m)}}}class U{constructor(){v(this,"runInPause",!1)}update(e,m){const p=e.getResource("grabbedBall"),r=e.getResource("gravity");if(!r)return;const o=e.query([M,j]);for(const s of o){if(s===p)continue;e.getComponent(s,M).vel.add(r,m)}}}class z{constructor(){v(this,"runInPause",!1)}_computeCOM(e,m){var o,s;let p=0;const r=new A(0,0);for(const l of m){const c=(o=e.getComponent(l,b))==null?void 0:o.pos,a=((s=e.getComponent(l,x))==null?void 0:s.mass)??0;!c||!(a>0)||(r.add(c.clone().scale(a)),p+=a)}return p>0&&r.scale(1/p),{com:r,sumMass:p}}update(e,m){var r,o;const p=e.query([F]);if(!(!p||p.length===0))for(const s of p){const l=e.getComponent(s,F),c=l.members||[];if(c.length<2)continue;if(!l.restLocal){const{com:f}=this._computeCOM(e,c);l.restLocal=c.map(C=>{var y;const d=(y=e.getComponent(C,b))==null?void 0:y.pos;return d?d.clone().subtract(f):new A(0,0)})}const{com:a,sumMass:u}=this._computeCOM(e,c);if(!(u>0))continue;let h=0,g=0;for(let f=0;f<c.length;f++){const C=c[f],d=(r=e.getComponent(C,b))==null?void 0:r.pos,y=((o=e.getComponent(C,x))==null?void 0:o.mass)??0;if(!d||!(y>0))continue;const I=d.clone().subtract(a),k=l.restLocal[f]||new A(0,0);h+=y*(k.x*I.x+k.y*I.y),g+=y*(k.x*I.y-k.y*I.x)}const t=Math.atan2(g,h),n=Math.cos(t),P=Math.sin(t),S=Math.max(0,Math.min(1,l.stiffness??1));for(let f=0;f<c.length;f++){const C=c[f],d=e.getComponent(C,b);if(!d)continue;const y=l.restLocal[f]||new A(0,0),I=n*y.x-P*y.y+a.x,k=P*y.x+n*y.y+a.y,B=new A(I-d.pos.x,k-d.pos.y).scale(S);d.pos.add(B)}let $=t-(l.prevAngle||0);for(;$>Math.PI;)$-=2*Math.PI;for(;$<-Math.PI;)$+=2*Math.PI;if(Math.abs($)>0)for(let f=0;f<c.length;f++){const C=c[f],d=e.getComponent(C,R);d&&(d.angle+=$)}try{if(e.getResource&&e.getResource("debugAngles")){const f=c[0],C=e.getComponent(f,R);console.log("[RigidGroupSystem]",{angle:t,deltaAngle:$,o0:C?C.angle:null})}}catch{}l.prevAngle=t}}}class J{constructor(){v(this,"runInPause",!1)}update(e,m){const p=e.getResource("grabbedBall"),r=e.query([b,M]);for(const o of r){if(o===p)continue;const s=e.getComponent(o,b),l=e.getComponent(o,M);s.pos.add(l.vel,m)}}}class X{constructor(){v(this,"runInPause",!1)}update(e,m){const p=e.query([b,V]);for(const r of p){const o=e.getComponent(r,b);e.getComponent(r,V).pos.set(o.pos)}}}class Y{constructor(){v(this,"runInPause",!1)}update(e,m){const p=e.query([R,O]);for(const r of p){const o=e.getComponent(r,R),s=e.getComponent(r,O);o.angle+=s.angularVelocity*m}}}export{Y as A,U as G,J as M,X as P,z as R,W as a,D as b,N as c,L as d};
