import {
  CablePathComponent
} from './cable_joints_core.js';

export function createCablePaths(world, jointEntities = [], linkTypes = [], cw = [], spring_constant = 1e6, userStored = null) {
  const createdPathEntityIds = [];

  if (linkTypes.length !== jointEntities.length + 1) {
    console.warn("createCablePaths: linkTypes.length must be jointEntities.length + 1. Aborting.");
    return createdPathEntityIds;
  }
  if (cw.length !== linkTypes.length) {
    console.warn("createCablePaths: cw.length must be linkTypes.length. Aborting.");
    return createdPathEntityIds;
  }
  if (userStored && userStored.length !== linkTypes.length) {
    console.warn("createCablePaths: userStored.length must be linkTypes.length if provided. Aborting.");
    return createdPathEntityIds;
  }

  if (jointEntities.length === 0) {
    const pathEntityId = world.createEntity();
    const pathComponent = new CablePathComponent(world, [], linkTypes, cw, spring_constant, userStored);
    world.addComponent(pathEntityId, pathComponent);
    createdPathEntityIds.push(pathEntityId);
    return createdPathEntityIds;
  }

  let currentPathJoints = [];
  let currentPathLinkTypes = [];
  let currentPathCw = [];
  let currentPathStored = userStored ? [] : null;

  currentPathLinkTypes.push(linkTypes[0]);
  currentPathCw.push(cw[0]);
  if (userStored) {
    currentPathStored.push(userStored[0]);
  }

  for (let i = 0; i < jointEntities.length; i++) {
    const jointId = jointEntities[i];
    const linkTypeAfterJoint = linkTypes[i + 1];
    const cwAfterJoint = cw[i + 1];
    const storedAfterJoint = userStored ? userStored[i + 1] : null;

    currentPathJoints.push(jointId);
    currentPathLinkTypes.push(linkTypeAfterJoint);
    currentPathCw.push(cwAfterJoint);
    if (userStored) {
      currentPathStored.push(storedAfterJoint);
    }

    if (linkTypeAfterJoint === 'attachment' && (i + 1) < (linkTypes.length - 1)) {
      const pathEntityId = world.createEntity();
      const pathComponent = new CablePathComponent(world, currentPathJoints, currentPathLinkTypes, currentPathCw, spring_constant, currentPathStored);
      world.addComponent(pathEntityId, pathComponent);
      createdPathEntityIds.push(pathEntityId);

      currentPathJoints = [];
      currentPathLinkTypes = [linkTypeAfterJoint];
      currentPathCw = [cwAfterJoint];
      if (userStored) {
        currentPathStored = [storedAfterJoint];
      }
    }
  }

  if (currentPathJoints.length > 0 || (createdPathEntityIds.length === 0 && linkTypes.length > 0)) {
    const pathEntityId = world.createEntity();
    const pathComponent = new CablePathComponent(world, currentPathJoints, currentPathLinkTypes, currentPathCw, spring_constant, currentPathStored);
    world.addComponent(pathEntityId, pathComponent);
    createdPathEntityIds.push(pathEntityId);
  }

  return createdPathEntityIds;
}
