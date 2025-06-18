import Vector2 from './vector2.js';

import {
  CablePathComponent
} from './cable_joints_core.js';

export function createCablePaths(world, jointEntities = [], linkTypes = [], cw = [], spring_constant = 1e6, userStored = null) {
    const createdPathEntityIds = [];

    // Validate input array lengths relative to each other for a single conceptual path
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
        // This means linkTypes.length is 1 (due to validation above). A single link, no joints.
        // Create one CablePathComponent.
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

    // The first link type always starts the first path segment
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

        // Check if the link *after* the current joint is an 'attachment'
        // and if it's not the very last link in the overall path definition.
        // (i + 1) is the index of linkTypeAfterJoint in the original linkTypes array.
        // linkTypes.length - 1 is the index of the last link type in the original linkTypes array.
        if (linkTypeAfterJoint === 'attachment' && (i + 1) < (linkTypes.length - 1)) {
            // Finalize current path
            const pathEntityId = world.createEntity();
            const pathComponent = new CablePathComponent(world, currentPathJoints, currentPathLinkTypes, currentPathCw, spring_constant, currentPathStored);
            world.addComponent(pathEntityId, pathComponent);
            createdPathEntityIds.push(pathEntityId);

            // Start a new path
            currentPathJoints = [];
            // The 'attachment' link that caused the split starts the new path segment.
            currentPathLinkTypes = [linkTypeAfterJoint];
            currentPathCw = [cwAfterJoint];
            if (userStored) {
                currentPathStored = [storedAfterJoint];
            }
        }
    }

    // Add the last (or only remaining) path segment
    if (currentPathJoints.length > 0 || (createdPathEntityIds.length === 0 && linkTypes.length > 0)) {
        // The second condition (createdPathEntityIds.length === 0 && linkTypes.length > 0)
        // should not be strictly necessary due to the jointEntities.length === 0 check at the start,
        // but kept for robustness in case of unusual inputs that might bypass earlier checks
        // and result in no joints processed but still a valid single segment.
        const pathEntityId = world.createEntity();
        const pathComponent = new CablePathComponent(world, currentPathJoints, currentPathLinkTypes, currentPathCw, spring_constant, currentPathStored);
        world.addComponent(pathEntityId, pathComponent);
        createdPathEntityIds.push(pathEntityId);
    }

    return createdPathEntityIds;
}

