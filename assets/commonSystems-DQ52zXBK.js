var q=Object.defineProperty;var D=(a,e,i)=>e in a?q(a,e,{enumerable:!0,configurable:!0,writable:!0,value:i}):a[e]=i;var g=(a,e,i)=>D(a,typeof e!="symbol"?e+"":e,i);import{c as $,j as I,O as P,k,d as b,A,D as j,M as R,g as V,V as W,G as L}from"./renderSystem-e7hQh9L-.js";function z(a){const e=a.systems.find(c=>c.constructor.name==="RenderSystem"),i=e?e.viewScaleMultiplier:1,m=e?e.viewOffsetX_sim:0,r=e?e.viewOffsetY_sim:0;let s=`
// --- Generated Error Test Case ---
testGeneratedError: {
  // Viewport settings from the time of the error
  viewport: { scale: ${i}, offsetX: ${m}, offsetY: ${r} },
  run: function() {
    const testName = "Generated Error Case";
    const world = new World();
    window.testWorld = world; // Set global world
    world.setResource('dt', ${a.getResource("dt")});
    world.setResource('debugRenderPoints', {});
    world.setResource('simWidth', ${a.getResource("simWidth")});
    world.setResource('simHeight', ${a.getResource("simHeight")});

    // --- Resources ---
`;const o=a.getResource("gravity");o&&(s+=`    world.setResource('gravity', new Vector2(${o.x}, ${o.y}));
`);const l=a.getResource("errorState");l&&(s+=`    world.setResource('errorState', new SimulationErrorStateComponent(${l.hasError}));
`),s+=`
    // --- Entities and Components ---
`;const u=new Map;for(const c of a.entities.keys()){const p=`entity${c}`;u.set(c,p),s+=`    const ${p} = world.createEntity(); // Original ID: ${c}
`}s+=`
`;for(const c of a.entities.keys()){const p=u.get(c),f=a.entities.get(c);if(f){for(const C of f){const t=a.getComponent(c,C);if(!t)continue;let n=`    world.addComponent(${p}, new ${C.name}(`;switch(C.name){case"PositionComponent":case"PrevFinalPosComponent":n+=`${t.pos.x}, ${t.pos.y}`;break;case"VelocityComponent":n+=`${t.vel.x}, ${t.vel.y}`;break;case"RadiusComponent":n+=`${t.radius}`;break;case"MassComponent":n+=`${t.mass}`;break;case"ObstaclePushComponent":n+=`${t.pushVel}`;break;case"ScoreComponent":n+=`${t.value}`;break;case"RestitutionComponent":n+=`${t.restitution}`;break;case"CoefficientOfFrictionComponent":n+=`${t.mu}`;break;case"OrientationComponent":case"PrevFinalOrientationComponent":n+=`${t.angle}`;break;case"AngularVelocityComponent":n+=`${t.angularVelocity}`;break;case"MomentOfInertiaComponent":n+=`${t.inertia}`;break;case"RenderableComponent":n+=`'${t.shape}', '${t.color}'`;break;case"FlipperStateComponent":n+=`${t.length}, ${t.restAngle},  ${t.sign*t.maxRotation}, ${t.angularVelocity}`;break;case"FlipperTipComponent":const d=u.get(t.flipperEntityId);d?n+=d:(console.warn(`Could not find variable name for flipper entity ${t.flipperEntityId} in FlipperTipComponent for entity ${c}`),n=`    // Skipped FlipperTipComponent for ${p} due to missing entity mapping
`);break;case"CableJointComponent":const y=u.get(t.entityA),h=u.get(t.entityB);!y||!h?(console.warn(`Could not find variable names for entities ${t.entityA} or ${t.entityB} in joint ${c}`),n=`    // Skipped CableJointComponent for ${p} due to missing entity mapping
`):t.attachmentPointA_world&&t.attachmentPointB_world?(n+=`${y}, ${h}, ${t.restLength}, `,n+=`new Vector2(${t.attachmentPointA_world.x}, ${t.attachmentPointA_world.y}), `,n+=`new Vector2(${t.attachmentPointB_world.x}, ${t.attachmentPointB_world.y})`):n=`    // Skipped CableJointComponent for ${p}: could not find local attachment point properties (e.g., 'attachmentPointA_world').
`;break;case"PauseStateComponent":n+=t.paused;break;case"SimulationErrorStateComponent":n+=t.hasError;break;case"BorderComponent":const v=t.points.map(S=>`new Vector2(${S.x}, ${S.y})`).join(", ");n+=`[${v}]`;break;case"CablePathComponent":n=`
`;break;case"GravityAffectedComponent":case"CableLinkComponent":case"BallTagComponent":case"FlipperTagComponent":case"ObstacleTagComponent":case"ScoredTagComponent":break;default:console.warn(`Unhandled component type for serialization: ${C.name}`),n=`    // Skipped unknown component ${C.name} for ${p}
`}n.endsWith(`
`)||(n+=`));
`),s+=n}s+=`
`}}for(const c of a.entities.keys()){const p=u.get(c),f=a.entities.get(c);if(f)for(const C of f){const t=a.getComponent(c,C);if(!t)continue;let n="";if(C.name==="CablePathComponent"){n+=`    world.addComponent(${p}, new ${C.name}(`;const d=t.jointEntities.map(y=>u.get(y)).filter(Boolean);if(d.length!==t.jointEntities.length)console.warn(`Could not find variable names for all joints in path ${c}`),n=`    // Skipped CablePathComponent for ${p} due to missing joint mapping
`;else{n+=`world, [${d.join(", ")}], `,n+=`[${t.linkTypes.map(y=>`'${y}'`).join(", ")}], `,n+=`[${t.cw.join(", ")}], `,n+=`${t.spring_constant}, `,n+=`[${t.stored.join(", ")}]`,n+=`));
`,n+=`    const pathComp_${p} = world.getComponent(${p}, CablePathComponent);
`,n+=`    pathComp_${p}.totalRestLength = ${t.totalRestLength};
`,s+=n;continue}}!n.endsWith(`
`)&&n!==""&&(n+=`));
`),s+=n}}return s+=`
`,s+=`
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
`,s}class J{constructor(){g(this,"runInPause",!1)}update(e,i){const m=e.query([P,k]);for(const r of m){const s=e.getComponent(r,P),o=e.getComponent(r,k);o.angle=s.angle}}}class Y{constructor(){g(this,"runInPause",!1)}_normalizeAngle(e){for(;e>Math.PI;)e-=2*Math.PI;for(;e<-Math.PI;)e+=2*Math.PI;return e}update(e,i){const m=e.getResource("grabbedBall"),r=e.query([P,A,k,V]);if(!(i<=1e-9))for(const o of r){if(o===m)continue;const l=e.getComponent(o,V);if(l&&l.invInertia<=0)continue;const u=e.getComponent(o,P),c=e.getComponent(o,A),p=e.getComponent(o,k),f=u.angle,C=p.angle;let t=this._normalizeAngle(f-C);c.angularVelocity=t/i}}}class H{constructor(){g(this,"runInPause",!1)}update(e,i){const m=e.getResource("grabbedBall"),r=e.query([$,b,I,R]);if(!(i<=1e-9))for(const o of r){if(o===m)continue;const l=e.getComponent(o,R);if(l&&l.mass<=0)continue;const u=e.getComponent(o,$),c=e.getComponent(o,b),p=e.getComponent(o,I);c.vel.subtractVectors(u.pos,p.pos).scale(1/i)}}}class K{constructor(){g(this,"runInPause",!1)}update(e,i){const m=e.getResource("grabbedBall"),r=e.getResource("gravity");if(!r)return;const s=e.query([b,L]);for(const o of s){if(o===m)continue;e.getComponent(o,b).vel.add(r,i)}}}class Q{constructor(){g(this,"runInPause",!1)}update(e,i){const m=e.query([j]),r=1e-9;for(const s of m){const o=e.getComponent(s,j),l=o.entityA,u=o.entityB,c=e.getComponent(l,$),p=e.getComponent(u,$);if(!c||!p)continue;const f=c.pos,C=p.pos,t=e.getComponent(l,R),n=t&&t.mass>0?1/t.mass:0,d=e.getComponent(l,V),y=d?d.invInertia:0,h=e.getComponent(u,R),v=h&&h.mass>0?1/h.mass:0,S=e.getComponent(u,V),T=S?S.invInertia:0;if(n+v+y+T<=r)continue;const O=new W().subtractVectors(C,f),B=O.length();if(B<=r)continue;const x=O.clone().scale(1/B),G=B-o.restLength,_=o.compliance/(i*i),F=n+v+_;if(F<=r)continue;const M=(-G-_*o.lambda)/F;o.lambda+=M;const E=x.clone().scale(M);if(n>0){const w=E.clone().scale(-n);f.add(w),e.getComponent(l,b)}if(v>0){const w=E.clone().scale(v);C.add(w),e.getComponent(u,b)}}}}class Z{constructor(){g(this,"runInPause",!1)}update(e,i){const m=e.getResource("grabbedBall"),r=e.query([$,b]);for(const s of r){if(s===m)continue;const o=e.getComponent(s,$),l=e.getComponent(s,b);o.pos.add(l.vel,i)}}}class ee{constructor(){g(this,"runInPause",!1)}update(e,i){const m=e.query([$,I]);for(const r of m){const s=e.getComponent(r,$);e.getComponent(r,I).pos.set(s.pos)}}}class te{constructor(){g(this,"runInPause",!1)}update(e,i){const m=e.query([P,A]);for(const r of m){const s=e.getComponent(r,P),o=e.getComponent(r,A);s.angle+=o.angularVelocity*i}}}export{te as A,K as G,Z as M,ee as P,Q as X,J as a,H as b,Y as c,z as d};
