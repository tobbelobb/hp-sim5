var D=Object.defineProperty;var W=(r,e,m)=>e in r?D(r,e,{enumerable:!0,configurable:!0,writable:!0,value:m}):r[e]=m;var I=(r,e,m)=>W(r,typeof e!="symbol"?e+"":e,m);import{P as b,a as B,O as A,f as O,b as k,G as N,A as _,M as V,V as M,p as T,D as L,e as x}from"./cable_joints_core-D81SdZSN.js";function J(r){const e=r.systems.find(i=>i.constructor.name==="RenderSystem"),m=e?e.viewScaleMultiplier:1,l=e?e.viewOffsetX_sim:0,c=e?e.viewOffsetY_sim:0;let s=`
// --- Generated Error Test Case ---
testGeneratedError: {
  // Viewport settings from the time of the error
  viewport: { scale: ${m}, offsetX: ${l}, offsetY: ${c} },
  run: function() {
    const testName = "Generated Error Case";
    const world = new World();
    window.testWorld = world; // Set global world
    world.setResource('dt', ${r.getResource("dt")});
    world.setResource('debugRenderPoints', {});
    world.setResource('simWidth', ${r.getResource("simWidth")});
    world.setResource('simHeight', ${r.getResource("simHeight")});

    // --- Resources ---
`;const o=r.getResource("gravity");o&&(s+=`    world.setResource('gravity', new Vector2(${o.x}, ${o.y}));
`);const p=r.getResource("errorState");p&&(s+=`    world.setResource('errorState', new SimulationErrorStateComponent(${p.hasError}));
`),s+=`
    // --- Entities and Components ---
`;const a=new Map;for(const i of r.entities.keys()){const f=`entity${i}`;a.set(i,f),s+=`    const ${f} = world.createEntity(); // Original ID: ${i}
`}s+=`
`;for(const i of r.entities.keys()){const f=a.get(i),$=r.entities.get(i);if($){for(const g of $){const t=r.getComponent(i,g);if(!t)continue;let n=`    world.addComponent(${f}, new ${g.name}(`;switch(g.name){case"PositionComponent":case"PrevFinalPosComponent":n+=`${t.pos.x}, ${t.pos.y}`;break;case"VelocityComponent":n+=`${t.vel.x}, ${t.vel.y}`;break;case"RadiusComponent":n+=`${t.radius}`;break;case"MassComponent":n+=`${t.mass}`;break;case"ObstaclePushComponent":n+=`${t.pushVel}`;break;case"ScoreComponent":n+=`${t.value}`;break;case"RestitutionComponent":n+=`${t.restitution}`;break;case"CoefficientOfFrictionComponent":n+=`${t.mu}`;break;case"OrientationComponent":case"PrevFinalOrientationComponent":n+=`${t.angle}`;break;case"AngularVelocityComponent":n+=`${t.angularVelocity}`;break;case"MomentOfInertiaComponent":n+=`${t.inertia}`;break;case"RenderableComponent":n+=`'${t.shape}', '${t.color}'`;break;case"FlipperStateComponent":n+=`${t.length}, ${t.restAngle},  ${t.sign*t.maxRotation}, ${t.angularVelocity}`;break;case"FlipperTipComponent":const v=a.get(t.flipperEntityId);v?n+=v:(console.warn(`Could not find variable name for flipper entity ${t.flipperEntityId} in FlipperTipComponent for entity ${i}`),n=`    // Skipped FlipperTipComponent for ${f} due to missing entity mapping
`);break;case"CableJointComponent":const P=a.get(t.entityA),y=a.get(t.entityB);!P||!y?(console.warn(`Could not find variable names for entities ${t.entityA} or ${t.entityB} in joint ${i}`),n=`    // Skipped CableJointComponent for ${f} due to missing entity mapping
`):t.attachmentPointA_world&&t.attachmentPointB_world?(n+=`${P}, ${y}, ${t.restLength}, `,n+=`new Vector2(${t.attachmentPointA_world.x}, ${t.attachmentPointA_world.y}), `,n+=`new Vector2(${t.attachmentPointB_world.x}, ${t.attachmentPointB_world.y})`):n=`    // Skipped CableJointComponent for ${f}: could not find local attachment point properties (e.g., 'attachmentPointA_world').
`;break;case"PauseStateComponent":n+=t.paused;break;case"SimulationErrorStateComponent":n+=t.hasError;break;case"BorderComponent":const u=t.points.map(C=>`new Vector2(${C.x}, ${C.y})`).join(", ");n+=`[${u}]`;break;case"CablePathComponent":n=`
`;break;case"GravityAffectedComponent":case"CableLinkComponent":case"BallTagComponent":case"FlipperTagComponent":case"ObstacleTagComponent":case"ScoredTagComponent":break;default:console.warn(`Unhandled component type for serialization: ${g.name}`),n=`    // Skipped unknown component ${g.name} for ${f}
`}n.endsWith(`
`)||(n+=`));
`),s+=n}s+=`
`}}for(const i of r.entities.keys()){const f=a.get(i),$=r.entities.get(i);if($)for(const g of $){const t=r.getComponent(i,g);if(!t)continue;let n="";if(g.name==="CablePathComponent"){n+=`    world.addComponent(${f}, new ${g.name}(`;const v=t.jointEntities.map(P=>a.get(P)).filter(Boolean);if(v.length!==t.jointEntities.length)console.warn(`Could not find variable names for all joints in path ${i}`),n=`    // Skipped CablePathComponent for ${f} due to missing joint mapping
`;else{n+=`world, [${v.join(", ")}], `,n+=`[${t.linkTypes.map(P=>`'${P}'`).join(", ")}], `,n+=`[${t.cw.join(", ")}], `,n+=`${t.spring_constant}, `,n+=`[${t.stored.join(", ")}]`,n+=`));
`,n+=`    const pathComp_${f} = world.getComponent(${f}, CablePathComponent);
`,n+=`    pathComp_${f}.totalRestLength = ${t.totalRestLength};
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
`,s}class Y{constructor(){I(this,"runInPause",!1)}update(e,m){const l=e.query([A,O]);for(const c of l){const s=e.getComponent(c,A),o=e.getComponent(c,O);o.angle=s.angle}}}class H{constructor(){I(this,"runInPause",!1)}_normalizeAngle(e){for(;e>Math.PI;)e-=2*Math.PI;for(;e<-Math.PI;)e+=2*Math.PI;return e}update(e,m){const l=e.getResource("grabbedBall"),c=e.query([A,_,O,x]);if(!(m<=1e-9))for(const o of c){if(o===l)continue;const p=e.getComponent(o,x);if(p&&p.invInertia<=0)continue;const a=e.getComponent(o,A),i=e.getComponent(o,_),f=e.getComponent(o,O),$=a.angle,g=f.angle;let t=this._normalizeAngle($-g);i.angularVelocity=t/m}}}class K{constructor(){I(this,"runInPause",!1)}update(e,m){const l=e.getResource("grabbedBall"),c=e.query([b,k,B,V]);if(!(m<=1e-9))for(const o of c){if(o===l)continue;const p=e.getComponent(o,V);if(p&&p.mass<=0)continue;const a=e.getComponent(o,b),i=e.getComponent(o,k),f=e.getComponent(o,B);i.vel.subtractVectors(a.pos,f.pos).scale(1/m)}}}class Q{constructor(){I(this,"runInPause",!1)}update(e,m){const l=e.getResource("grabbedBall"),c=e.getResource("gravity");if(!c)return;const s=e.query([k,N]);for(const o of s){if(o===l)continue;e.getComponent(o,k).vel.add(c,m)}}}class Z{constructor(){I(this,"runInPause",!1)}update(e,m){const l=e.query([L]),c=1e-9;for(const s of l){const o=e.getComponent(s,L),p=o.entityA,a=o.entityB,i=e.getComponent(p,b),f=e.getComponent(a,b);if(!i||!f)continue;const $=i.pos,g=f.pos,t=e.getComponent(p,V),n=t&&t.mass>0?1/t.mass:0,v=e.getComponent(p,x),P=v?v.invInertia:0,y=e.getComponent(a,V),u=y&&y.mass>0?1/y.mass:0,C=e.getComponent(a,x),d=C?C.invInertia:0;if(n+u+P+d<=c)continue;const h=new M().subtractVectors(g,$),S=h.length();if(S<=c)continue;const R=h.clone().scale(1/S),E=S-o.restLength,w=o.compliance/(m*m),G=n+u+w;if(G<=c)continue;const j=(-E-w*o.lambda)/G;o.lambda+=j;const q=R.clone().scale(j);if(n>0){const F=q.clone().scale(-n);$.add(F),e.getComponent(p,k)}if(u>0){const F=q.clone().scale(u);g.add(F),e.getComponent(a,k)}}}}class ee{constructor(){I(this,"runInPause",!1)}_computeCOM(e,m){var s,o;let l=0;const c=new M(0,0);for(const p of m){const a=(s=e.getComponent(p,b))==null?void 0:s.pos,i=((o=e.getComponent(p,V))==null?void 0:o.mass)??0;!a||!(i>0)||(c.add(a.clone().scale(i)),l+=i)}return l>0&&c.scale(1/l),{com:c,sumMass:l}}update(e,m){var c,s;const l=e.query([T]);if(!(!l||l.length===0))for(const o of l){const p=e.getComponent(o,T),a=p.members||[];if(a.length<2)continue;if(!p.restLocal){const{com:u}=this._computeCOM(e,a);p.restLocal=a.map(C=>{var h;const d=(h=e.getComponent(C,b))==null?void 0:h.pos;return d?d.clone().subtract(u):new M(0,0)})}const{com:i,sumMass:f}=this._computeCOM(e,a);if(!(f>0))continue;let $=0,g=0;for(let u=0;u<a.length;u++){const C=a[u],d=(c=e.getComponent(C,b))==null?void 0:c.pos,h=((s=e.getComponent(C,V))==null?void 0:s.mass)??0;if(!d||!(h>0))continue;const S=d.clone().subtract(i),R=p.restLocal[u]||new M(0,0);$+=h*(R.x*S.x+R.y*S.y),g+=h*(R.x*S.y-R.y*S.x)}const t=Math.atan2(g,$),n=Math.cos(t),v=Math.sin(t),P=Math.max(0,Math.min(1,p.stiffness??1));for(let u=0;u<a.length;u++){const C=a[u],d=e.getComponent(C,b);if(!d)continue;const h=p.restLocal[u]||new M(0,0),S=n*h.x-v*h.y+i.x,R=v*h.x+n*h.y+i.y,E=new M(S-d.pos.x,R-d.pos.y).scale(P);d.pos.add(E)}let y=t-(p.prevAngle||0);for(;y>Math.PI;)y-=2*Math.PI;for(;y<-Math.PI;)y+=2*Math.PI;if(Math.abs(y)>0)for(let u=0;u<a.length;u++){const C=a[u],d=e.getComponent(C,A);d&&(d.angle+=y)}try{if(e.getResource&&e.getResource("debugAngles")){const u=a[0],C=e.getComponent(u,A);console.log("[RigidGroupSystem]",{angle:t,deltaAngle:y,o0:C?C.angle:null})}}catch{}p.prevAngle=t}}}class te{constructor(){I(this,"runInPause",!1)}update(e,m){const l=e.getResource("grabbedBall"),c=e.query([b,k]);for(const s of c){if(s===l)continue;const o=e.getComponent(s,b),p=e.getComponent(s,k);o.pos.add(p.vel,m)}}}class ne{constructor(){I(this,"runInPause",!1)}update(e,m){const l=e.query([b,B]);for(const c of l){const s=e.getComponent(c,b);e.getComponent(c,B).pos.set(s.pos)}}}class oe{constructor(){I(this,"runInPause",!1)}update(e,m){const l=e.query([A,_]);for(const c of l){const s=e.getComponent(c,A),o=e.getComponent(c,_);s.angle+=o.angularVelocity*m}}}export{oe as A,Q as G,te as M,ne as P,ee as R,Z as X,Y as a,K as b,H as c,J as d};
