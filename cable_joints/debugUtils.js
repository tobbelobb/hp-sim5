export function dumpWorldState(world) {
  const renderSystem = world.getResource('renderSystem');
  const viewportScale = renderSystem ? renderSystem.viewScaleMultiplier : 1.0;
  const viewportOffsetX = renderSystem ? renderSystem.viewOffsetX_sim : 0.0;
  const viewportOffsetY = renderSystem ? renderSystem.viewOffsetY_sim : 0.0;

  let dump = `
// --- Generated Error Test Case ---
testGeneratedError: {
  // Viewport settings from the time of the error
  viewport: { scale: ${viewportScale}, offsetX: ${viewportOffsetX}, offsetY: ${viewportOffsetY} },
  run: function() {
    const testName = "Generated Error Case";
    const world = new World();
    window.testWorld = world; // Set global world
    world.setResource('dt', ${world.getResource('dt')});
    world.setResource('debugRenderPoints', {});
    world.setResource('simWidth', ${world.getResource('simWidth')});
    world.setResource('simHeight', ${world.getResource('simHeight')});

    // --- Resources ---
`;
  // Dump simple resources (add more as needed)
  const gravity = world.getResource('gravity');
  if (gravity) {
    dump += `    world.setResource('gravity', new Vector2(${gravity.x}, ${gravity.y}));\n`;
  }
  const errorState = world.getResource('errorState');
  if (errorState) {
    dump += `    world.setResource('errorState', new SimulationErrorStateComponent(${errorState.hasError}));\n`;
  }
  // Note: pauseState is usually set by the test runner or UI
  // Note: renderSystem is created by the test runner

  dump += `
    // --- Entities and Components ---
`;
  const entityVarMap = new Map(); // Map original entityId to generated variable name

  // First pass: Create entities and map IDs
  for (const entityId of world.entities.keys()) {
    const varName = `entity${entityId}`;
    entityVarMap.set(entityId, varName);
    dump += `    const ${varName} = world.createEntity(); // Original ID: ${entityId}\n`;
  }
  dump += "\n"; // Add a newline for separation

  // Second pass: Add components
  for (const entityId of world.entities.keys()) {
    const varName = entityVarMap.get(entityId);
    const componentClasses = world.entities.get(entityId);
    if (!componentClasses) continue;

    for (const componentClass of componentClasses) {
      const component = world.getComponent(entityId, componentClass);
      if (!component) continue;

      let addCompStr = `    world.addComponent(${varName}, new ${componentClass.name}(`;

      // Serialize component data based on type
      switch (componentClass.name) {
        case 'PositionComponent':
          addCompStr += `${component.pos.x}, ${component.pos.y}`;
          break;
        case 'VelocityComponent':
          addCompStr += `${component.vel.x}, ${component.vel.y}`;
          break;
        case 'RadiusComponent':
          addCompStr += `${component.radius}`;
          break;
        case 'MassComponent':
          addCompStr += `${component.mass}`;
          break;
        case 'ObstaclePushComponent':
          addCompStr += `${component.pushVel}`;
          break;
        case 'ScoreComponent':
          addCompStr += `${component.value}`;
          break;
        case 'RestitutionComponent':
          addCompStr += `${component.restitution}`;
          break;
        case 'RenderableComponent':
          addCompStr += `'${component.shape}', '${component.color}'`;
          break;
        case 'FlipperStateComponent':
          addCompStr += `${component.length}, ${component.restAngle},  ${component.maxRotation}, ${component.angularVelocity}`;
          break;
        case 'CableJointComponent':
          const entityAVar = entityVarMap.get(component.entityA);
          const entityBVar = entityVarMap.get(component.entityB);
          if (!entityAVar || !entityBVar) {
            console.warn(`Could not find variable names for entities ${component.entityA} or ${component.entityB} in joint ${entityId}`);
            addCompStr = `    // Skipped CableJointComponent for ${varName} due to missing entity mapping\n`;
          } else {
            addCompStr += `${entityAVar}, ${entityBVar}, ${component.restLength}, `;
            addCompStr += `new Vector2(${component.attachmentPointA_world.x}, ${component.attachmentPointA_world.y}), `;
            addCompStr += `new Vector2(${component.attachmentPointB_world.x}, ${component.attachmentPointB_world.y})`;
          }
          break;
        case 'PauseStateComponent': // Usually a resource, but handle if added to entity
          addCompStr += component.paused;
          break;
        case 'SimulationErrorStateComponent': // Usually a resource, but handle if added to entity
          addCompStr += component.hasError;
          break;
        case 'BorderComponent':
          const pointsStr = component.points.map(p => `new Vector2(${p.x}, ${p.y})`).join(', ');
          addCompStr += `[${pointsStr}]`;
          break;
        case 'CablePathComponent': // Handled separately below
          break;
        case 'GravityAffectedComponent':
        case 'CableLinkComponent':
        case 'BallTagComponent':
        case 'FlipperTagComponent':
        case 'ObstacleTagComponent':
          break;
        default:
          console.warn(`Unhandled component type for serialization: ${componentClass.name}`);
          addCompStr = `    // Skipped unknown component ${componentClass.name} for ${varName}\n`;

      }
      if (!addCompStr.endsWith('\n')) { // Avoid adding extra parenthesis if skipped or handled above
        addCompStr += '));\n';
      }
      dump += addCompStr;
    }
    dump += "\n"; // Add a newline between entities
  }
  for (const entityId of world.entities.keys()) {
    const varName = entityVarMap.get(entityId);
    const componentClasses = world.entities.get(entityId);
    if (!componentClasses) continue;

    for (const componentClass of componentClasses) {
      const component = world.getComponent(entityId, componentClass);
      if (!component) continue;
      let addCompStr = "";
      // Register paths last. They depend on everything else being defined first
      if (componentClass.name === 'CablePathComponent') {
        addCompStr += `    world.addComponent(${varName}, new ${componentClass.name}(`;
        const jointVars = component.jointEntities.map(id => entityVarMap.get(id)).filter(Boolean);
        if (jointVars.length !== component.jointEntities.length) {
          console.warn(`Could not find variable names for all joints in path ${entityId}`);
          addCompStr = `    // Skipped CablePathComponent for ${varName} due to missing joint mapping\n`;
        } else {
          addCompStr += `world, [${jointVars.join(', ')}], `;
          addCompStr += `[${component.linkTypes.map(t => `'${t}'`).join(', ')}], `;
          addCompStr += `[${component.cw.join(', ')}]`;
          // Need to manually set stored and totalRestLength after creation in the test
          addCompStr += `));\n`;
          addCompStr += `    const pathComp_${varName} = world.getComponent(${varName}, CablePathComponent);\n`;
          addCompStr += `    pathComp_${varName}.stored = [${component.stored.join(', ')}];\n`;
          addCompStr += `    pathComp_${varName}.totalRestLength = ${component.totalRestLength};\n`;
          addCompStr += `    pathComp_${varName}.spring_constant = ${component.spring_constant};\n`;
          dump += addCompStr;
          continue; // Skip the default closing parenthesis below
        }
      }
      if (!addCompStr.endsWith('\n') && addCompStr !== "") { // Avoid adding extra parenthesis if skipped or handled above
        addCompStr += '));\n';
      }
      dump += addCompStr;
    }
  }
  dump += "\n"; // Add a newline between entities

  dump += `
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
`;
  return dump;
}
