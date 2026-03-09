function h(s){const c=s.systems.find(n=>n.constructor.name==="RenderSystem"),u=c?c.viewScaleMultiplier:1,g=c?c.viewOffsetX_sim:0,y=c?c.viewOffsetY_sim:0;let r=`
// --- Generated Error Test Case ---
testGeneratedError: {
  // Viewport settings from the time of the error
  viewport: { scale: ${u}, offsetX: ${g}, offsetY: ${y} },
  run: function() {
    const testName = "Generated Error Case";
    const world = new World();
    window.testWorld = world; // Set global world
    world.setResource('dt', ${s.getResource("dt")});
    world.setResource('debugRenderPoints', {});
    world.setResource('simWidth', ${s.getResource("simWidth")});
    world.setResource('simHeight', ${s.getResource("simHeight")});

    // --- Resources ---
`;const d=s.getResource("gravity");d&&(r+=`    world.setResource('gravity', new Vector2(${d.x}, ${d.y}));
`);const f=s.getResource("errorState");f&&(r+=`    world.setResource('errorState', new SimulationErrorStateComponent(${f.hasError}));
`),r+=`
    // --- Entities and Components ---
`;const i=new Map;for(const n of s.entities.keys()){const o=`entity${n}`;i.set(n,o),r+=`    const ${o} = world.createEntity(); // Original ID: ${n}
`}r+=`
`;for(const n of s.entities.keys()){const o=i.get(n),l=s.entities.get(n);if(l){for(const a of l){const e=s.getComponent(n,a);if(!e)continue;let t=`    world.addComponent(${o}, new ${a.name}(`;switch(a.name){case"PositionComponent":case"PrevFinalPosComponent":t+=`${e.pos.x}, ${e.pos.y}`;break;case"VelocityComponent":t+=`${e.vel.x}, ${e.vel.y}`;break;case"RadiusComponent":t+=`${e.radius}`;break;case"MassComponent":t+=`${e.mass}`;break;case"ObstaclePushComponent":t+=`${e.pushVel}`;break;case"ScoreComponent":t+=`${e.value}`;break;case"RestitutionComponent":t+=`${e.restitution}`;break;case"CoefficientOfFrictionComponent":t+=`${e.mu}`;break;case"OrientationComponent":case"PrevFinalOrientationComponent":t+=`${e.angle}`;break;case"AngularVelocityComponent":t+=`${e.angularVelocity}`;break;case"MomentOfInertiaComponent":t+=`${e.inertia}`;break;case"RenderableComponent":t+=`'${e.shape}', '${e.color}'`;break;case"FlipperStateComponent":t+=`${e.length}, ${e.restAngle},  ${e.sign*e.maxRotation}, ${e.angularVelocity}`;break;case"FlipperTipComponent":const m=i.get(e.flipperEntityId);m?t+=m:(console.warn(`Could not find variable name for flipper entity ${e.flipperEntityId} in FlipperTipComponent for entity ${n}`),t=`    // Skipped FlipperTipComponent for ${o} due to missing entity mapping
`);break;case"CableJointComponent":const p=i.get(e.entityA),$=i.get(e.entityB);!p||!$?(console.warn(`Could not find variable names for entities ${e.entityA} or ${e.entityB} in joint ${n}`),t=`    // Skipped CableJointComponent for ${o} due to missing entity mapping
`):e.attachmentPointA_world&&e.attachmentPointB_world?(t+=`${p}, ${$}, ${e.restLength}, `,t+=`new Vector2(${e.attachmentPointA_world.x}, ${e.attachmentPointA_world.y}), `,t+=`new Vector2(${e.attachmentPointB_world.x}, ${e.attachmentPointB_world.y})`):t=`    // Skipped CableJointComponent for ${o}: could not find local attachment point properties (e.g., 'attachmentPointA_world').
`;break;case"PauseStateComponent":t+=e.paused;break;case"SimulationErrorStateComponent":t+=e.hasError;break;case"BorderComponent":const w=e.points.map(C=>`new Vector2(${C.x}, ${C.y})`).join(", ");t+=`[${w}]`;break;case"CablePathComponent":t=`
`;break;case"GravityAffectedComponent":case"CableLinkComponent":case"BallTagComponent":case"FlipperTagComponent":case"ObstacleTagComponent":case"ScoredTagComponent":break;default:console.warn(`Unhandled component type for serialization: ${a.name}`),t=`    // Skipped unknown component ${a.name} for ${o}
`}t.endsWith(`
`)||(t+=`));
`),r+=t}r+=`
`}}for(const n of s.entities.keys()){const o=i.get(n),l=s.entities.get(n);if(l)for(const a of l){const e=s.getComponent(n,a);if(!e)continue;let t="";if(a.name==="CablePathComponent"){t+=`    world.addComponent(${o}, new ${a.name}(`;const m=e.jointEntities.map(p=>i.get(p)).filter(Boolean);if(m.length!==e.jointEntities.length)console.warn(`Could not find variable names for all joints in path ${n}`),t=`    // Skipped CablePathComponent for ${o} due to missing joint mapping
`;else{t+=`world, [${m.join(", ")}], `,t+=`[${e.linkTypes.map(p=>`'${p}'`).join(", ")}], `,t+=`[${e.cw.join(", ")}], `,t+=`${e.spring_constant}, `,t+=`[${e.stored.join(", ")}]`,t+=`));
`,t+=`    const pathComp_${o} = world.getComponent(${o}, CablePathComponent);
`,t+=`    pathComp_${o}.totalRestLength = ${e.totalRestLength};
`,r+=t;continue}}!t.endsWith(`
`)&&t!==""&&(t+=`));
`),r+=t}}return r+=`
`,r+=`
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
`,r}export{h as d};
