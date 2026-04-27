import Vector3 from '../cable_joints_3d/vector3.js';
import Quaternion from '../cable_joints_3d/quaternion.js';
import {
  signedArcLengthOnWheel,
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
} from '../cable_joints_3d/geometry3.js';
import { getAttribute, getRelationship } from './stage.js';
import { parseUsdaAst } from './usda_parser.js';

const DEFAULT_INIT_POLICY = 'deriveMissing';
const DEFAULT_PLANE_NORMAL = new Vector3(0.0, 0.0, 1.0);
const MANUAL_MODE = 'manual';
const AUTO_MODE = 'auto';
const DERIVE_MISSING_POLICY = 'deriveMissing';
const DERIVE_ALL_POLICY = 'deriveAll';
const EPSILON = 1e-9;

function cloneAst(ast) {
  if (typeof structuredClone === 'function') {
    return structuredClone(ast);
  }
  return JSON.parse(JSON.stringify(ast));
}

function indexPrims(statements, parent = '', out = {}) {
  for (const stmt of statements ?? []) {
    if (stmt.type !== 'definition') {
      continue;
    }
    const path = `${parent}/${stmt.name}`.replace('//', '/');
    out[path] = stmt;
    if (stmt.statements) {
      indexPrims(stmt.statements, path, out);
    }
  }
  return out;
}

function normalizeToPlainArray(value) {
  if (Array.isArray(value)) {
    return value;
  }
  if (value && typeof ArrayBuffer !== 'undefined' && ArrayBuffer.isView?.(value)) {
    return Array.from(value);
  }
  return value;
}

function getParentPath(path) {
  if (typeof path !== 'string') {
    return null;
  }
  const trimmed = path.trim();
  if (!trimmed || trimmed === '/') {
    return null;
  }
  const separatorIndex = trimmed.lastIndexOf('/');
  if (separatorIndex <= 0) {
    return '/';
  }
  return trimmed.slice(0, separatorIndex);
}

function quoteString(value) {
  return `"${String(value).replace(/\\/g, '\\\\').replace(/"/g, '\\"')}"`;
}

function formatNumber(value) {
  if (!Number.isFinite(value)) {
    throw new Error(`Cannot serialize non-finite number: ${value}`);
  }
  if (Object.is(value, -0)) {
    return '0';
  }
  return Number(value).toString();
}

function serializeTuple(values) {
  return `(${values.map((value) => serializeScalar(value)).join(', ')})`;
}

function serializeScalar(value) {
  if (typeof value === 'number') {
    return formatNumber(value);
  }
  if (typeof value === 'string') {
    return quoteString(value);
  }
  if (value?.type === 'externalReferenceImport') {
    return `<${value.importPath}>`;
  }
  if (value?.type === 'externalReferenceSrc') {
    return `@${value.src ?? value.path ?? ''}@`;
  }
  throw new Error(`Unsupported USDA scalar value: ${JSON.stringify(value)}`);
}

function serializeValue(value, typeHint = null, assignmentContext = false) {
  if (Array.isArray(value)) {
    const shouldBracket = assignmentContext
      || typeHint?.endsWith('[]')
      || value.some((entry) => (
        entry?.type === 'externalReferenceImport'
        || entry?.type === 'externalReferenceSrc'
      ));
    if (shouldBracket) {
      return `[${value.map((entry) => (
        Array.isArray(entry) ? serializeTuple(entry) : serializeScalar(entry)
      )).join(', ')}]`;
    }
    if (value.length > 0 && Array.isArray(value[0])) {
      return `(${value.map((entry) => serializeTuple(entry)).join(', ')})`;
    }
    return serializeTuple(value);
  }
  return serializeScalar(value);
}

function serializeAssignment(assignment, indent) {
  const keyword = assignment.keyword ? `${assignment.keyword} ` : '';
  return `${indent}${keyword}${assignment.identifier} = ${serializeValue(assignment.value, null, true)}`;
}

function serializeDescriptor(descriptor, indent) {
  const assignments = descriptor?.assignments ?? [];
  if (assignments.length === 0) {
    return '';
  }
  const childIndent = `${indent}    `;
  return [
    '(',
    ...assignments.map((assignment) => serializeAssignment(assignment, childIndent)),
    `${indent})`,
  ].join('\n');
}

function serializeDeclaration(statement, indent) {
  const keyword = statement.keyword ? `${statement.keyword} ` : '';
  return `${indent}${keyword}${statement.defineType} ${statement.reference} = ${serializeValue(statement.value, statement.defineType, false)}`;
}

function serializeDefinition(statement, indent = '') {
  let header = `${indent}${statement.subType} ${statement.defType ? `${statement.defType} ` : ''}${quoteString(statement.name)}`;
  const descriptor = serializeDescriptor(statement.descriptor, indent);
  if (descriptor) {
    header += ` ${descriptor}`;
  }

  const childIndent = `${indent}    `;
  const body = (statement.statements ?? [])
    .map((child) => {
      if (child.type === 'definition') {
        return serializeDefinition(child, childIndent);
      }
      if (child.type === 'declaration') {
        return serializeDeclaration(child, childIndent);
      }
      throw new Error(`Unsupported USDA statement type: ${child.type}`);
    })
    .join('\n');

  if (!body) {
    return `${header}\n${indent}{\n${indent}}`;
  }
  return `${header}\n${indent}{\n${body}\n${indent}}`;
}

export function serializeUsdaSceneAst(ast) {
  const chunks = [`#usda ${ast.version ?? '1.0'}`];
  const descriptor = serializeDescriptor(ast.descriptor, '');
  if (descriptor) {
    chunks.push(descriptor);
  }
  if ((ast.statements ?? []).length > 0) {
    chunks.push(ast.statements.map((statement) => serializeDefinition(statement)).join('\n\n'));
  }
  return `${chunks.join('\n\n')}\n`;
}

function findDeclaration(prim, reference) {
  return prim?.statements?.find(
    (statement) => statement.type === 'declaration' && statement.reference === reference,
  ) ?? null;
}

function setDeclaration(prim, reference, value, defineType, keyword = 'custom') {
  const existing = findDeclaration(prim, reference);
  if (existing) {
    existing.value = value;
    if (defineType) {
      existing.defineType = defineType;
    }
    if (keyword !== undefined && keyword !== null) {
      existing.keyword = keyword;
    }
    return existing;
  }

  const declaration = {
    type: 'declaration',
    keyword,
    defineType,
    reference,
    value,
    descriptor: null,
  };
  if (!Array.isArray(prim.statements)) {
    prim.statements = [];
  }
  prim.statements.push(declaration);
  return declaration;
}

function toVector3(value) {
  if (!Array.isArray(value)) {
    return null;
  }
  return new Vector3(
    Number(value[0] ?? 0.0),
    Number(value[1] ?? 0.0),
    Number(value[2] ?? 0.0),
  );
}

function fromVector3(value) {
  return [value.x, value.y, value.z];
}

function readVectorAttribute(primNode, attributeName) {
  return toVector3(normalizeToPlainArray(getAttribute(primNode, attributeName)));
}

function readQuaternionAttribute(primNode, attributeName) {
  const rawValue = normalizeToPlainArray(getAttribute(primNode, attributeName));
  if (!Array.isArray(rawValue) || rawValue.length < 4) {
    return null;
  }
  const w = Number(rawValue[0]);
  const x = Number(rawValue[1]);
  const y = Number(rawValue[2]);
  const z = Number(rawValue[3]);
  if (![w, x, y, z].every(Number.isFinite)) {
    return null;
  }
  return new Quaternion(x, y, z, w).normalize();
}

function readRotateABCAttribute(primNode, attributeName, rotationOrder) {
  const rawValue = normalizeToPlainArray(getAttribute(primNode, attributeName));
  if (!Array.isArray(rawValue) || rawValue.length < 3) {
    return null;
  }
  const xDeg = Number(rawValue[0]);
  const yDeg = Number(rawValue[1]);
  const zDeg = Number(rawValue[2]);
  if (![xDeg, yDeg, zDeg].every(Number.isFinite)) {
    return null;
  }

  const degToRad = Math.PI / 180.0;
  const axisRotations = {
    X: new Quaternion().setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), xDeg * degToRad),
    Y: new Quaternion().setFromAxisAngle(new Vector3(0.0, 1.0, 0.0), yDeg * degToRad),
    Z: new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), zDeg * degToRad),
  };

  const combined = new Quaternion();
  for (const axisName of rotationOrder) {
    const axisRotation = axisRotations[axisName];
    if (!axisRotation) {
      return null;
    }
    combined.multiplyQuaternions(axisRotation, combined).normalize();
  }
  return combined;
}

function readRotationOpAttribute(primNode, opName) {
  if (typeof opName !== 'string') {
    return null;
  }
  const normalizedOpName = opName.startsWith('!invert!') ? opName.slice('!invert!'.length) : opName;
  const orientMatch = normalizedOpName.match(/^xformOp:orient(?:[:].+)?$/);
  if (orientMatch) {
    return readQuaternionAttribute(primNode, normalizedOpName);
  }
  const rotateMatch = normalizedOpName.match(/^xformOp:rotate(XYZ|XZY|YXZ|YZX|ZXY|ZYX)(?:[:].+)?$/);
  if (rotateMatch) {
    return readRotateABCAttribute(primNode, normalizedOpName, rotateMatch[1]);
  }
  return null;
}

function readOrientationAttribute(primNode) {
  const xformOpOrder = normalizeToPlainArray(getAttribute(primNode, 'xformOpOrder'));
  const rotationOrder = Array.isArray(xformOpOrder) ? xformOpOrder : [];
  if (rotationOrder.length > 0) {
    const combined = new Quaternion();
    let found = false;
    for (const opName of rotationOrder) {
      const opRotation = readRotationOpAttribute(primNode, opName);
      if (!opRotation) {
        continue;
      }
      combined.multiplyQuaternions(opRotation, combined).normalize();
      found = true;
    }
    if (found) {
      return combined;
    }
  }
  return (
    readQuaternionAttribute(primNode, 'xformOp:orient')
    || readRotateABCAttribute(primNode, 'xformOp:rotateXYZ', 'XYZ')
    || readRotateABCAttribute(primNode, 'xformOp:rotateXZY', 'XZY')
    || readRotateABCAttribute(primNode, 'xformOp:rotateYXZ', 'YXZ')
    || readRotateABCAttribute(primNode, 'xformOp:rotateYZX', 'YZX')
    || readRotateABCAttribute(primNode, 'xformOp:rotateZXY', 'ZXY')
    || readRotateABCAttribute(primNode, 'xformOp:rotateZYX', 'ZYX')
    || new Quaternion()
  );
}

function readAxisLocal(primNode) {
  const axis = (
    readVectorAttribute(primNode, 'spool:axisLocal')
    || readVectorAttribute(primNode, 'machine:axisLocal')
    || readVectorAttribute(primNode, 'physics:rotationAxis')
    || DEFAULT_PLANE_NORMAL.clone()
  );
  if (axis.lengthSq() <= EPSILON) {
    return DEFAULT_PLANE_NORMAL.clone();
  }
  return axis.normalize();
}

function getPrimWorldTransform(index, path, cache) {
  if (cache.has(path)) {
    return cache.get(path);
  }

  const prim = index[path];
  if (!prim) {
    const identity = { position: new Vector3(), orientation: new Quaternion() };
    cache.set(path, identity);
    return identity;
  }

  const localPosition = readVectorAttribute(prim, 'xformOp:translate') ?? new Vector3();
  const localOrientation = readOrientationAttribute(prim);
  const parentPath = getParentPath(path);
  const parentPrim = parentPath ? index[parentPath] : null;

  let position = localPosition.clone();
  let orientation = localOrientation.clone().normalize();

  if (parentPrim) {
    const parentTransform = getPrimWorldTransform(index, parentPath, cache);
    position = parentTransform.position
      .clone()
      .add(parentTransform.orientation.transformVector(localPosition));
    orientation = new Quaternion()
      .multiplyQuaternions(parentTransform.orientation, localOrientation)
      .normalize();
  }

  const transform = { position, orientation };
  cache.set(path, transform);
  return transform;
}

function computeWorldPoint(transform, localPoint) {
  return transform.position.clone().add(transform.orientation.transformVector(localPoint));
}

function computeLocalPoint(transform, worldPoint) {
  const relative = worldPoint.clone().subtract(transform.position);
  const inverse = transform.orientation.clone().conjugate().normalize();
  return inverse.transformVector(relative);
}

function projectPointToPlane(point, planePoint, planeNormal) {
  const normal = planeNormal.clone();
  if (normal.lengthSq() <= EPSILON) {
    return point.clone();
  }
  normal.normalize();
  return point.clone().subtract(normal, point.clone().subtract(planePoint).dot(normal));
}

function isRollingLink(linkType) {
  return linkType === 'rolling' || linkType === 'hybrid';
}

function isDerivableStoredLink(linkType) {
  return linkType === 'rolling';
}

function effectiveCW(clockwise, linkIndex, travellingFromCircle) {
  const base = Boolean(clockwise?.[linkIndex]);
  if (linkIndex === 0 && travellingFromCircle) {
    return !base;
  }
  return base;
}

function deriveRollingSidePoint(body, counterpart, linkType, clockwise, pointIsFirst) {
  if (!isRollingLink(linkType)) {
    return body.position.clone();
  }
  if (!(body.radius > EPSILON)) {
    throw new Error(`Cannot derive rolling tangent without radius on ${body.path}.`);
  }
  const projectedCounterpart = projectPointToPlane(counterpart.position, body.position, body.planeNormal);
  return pointIsFirst
    ? tangentFromSphereToPoint(
      projectedCounterpart,
      body.position,
      body.radius,
      body.planeNormal,
      clockwise,
    ).a_sphere
    : tangentFromPointToSphere(
      projectedCounterpart,
      body.position,
      body.radius,
      body.planeNormal,
      clockwise,
    ).a_sphere;
}

function normalizeInitPolicy(rawValue) {
  if (rawValue === MANUAL_MODE || rawValue === DERIVE_MISSING_POLICY || rawValue === DERIVE_ALL_POLICY) {
    return rawValue;
  }
  return DEFAULT_INIT_POLICY;
}

function normalizeStoredValues(pathPrim, expectedLength) {
  const authored = findDeclaration(pathPrim, 'cablePath:stored');
  if (!authored) {
    return new Array(expectedLength).fill(null);
  }
  const values = normalizeToPlainArray(authored.value);
  if (!Array.isArray(values) || values.length !== expectedLength) {
    throw new Error(
      `${pathPrim.name}: cablePath:stored must have ${expectedLength} entries when authored.`,
    );
  }
  return values.map((value) => Number(value));
}

function normalizeStoredModes(pathPrim, expectedLength) {
  const authored = findDeclaration(pathPrim, 'cablePath:storedMode');
  if (!authored) {
    return new Array(expectedLength).fill(null);
  }
  const values = normalizeToPlainArray(authored.value);
  if (!Array.isArray(values) || values.length !== expectedLength) {
    throw new Error(
      `${pathPrim.name}: cablePath:storedMode must have ${expectedLength} entries when authored.`,
    );
  }
  return values.map((value) => {
    if (value === MANUAL_MODE || value === AUTO_MODE) {
      return value;
    }
    throw new Error(`${pathPrim.name}: unsupported cablePath:storedMode value "${value}".`);
  });
}

function buildBodyContext(index, path, transformCache, bodyCache) {
  if (bodyCache.has(path)) {
    return bodyCache.get(path);
  }

  const prim = index[path];
  if (!prim) {
    throw new Error(`Missing body prim at ${path}.`);
  }

  const transform = getPrimWorldTransform(index, path, transformCache);
  const radiusValue = Number(getAttribute(prim, 'radius'));
  const radius = Number.isFinite(radiusValue) ? radiusValue : 0.0;
  const planeNormal = transform.orientation.transformVector(readAxisLocal(prim));
  const context = {
    path,
    prim,
    transform,
    position: transform.position.clone(),
    orientation: transform.orientation.clone(),
    radius,
    planeNormal: planeNormal.lengthSq() > EPSILON ? planeNormal.normalize() : DEFAULT_PLANE_NORMAL.clone(),
  };
  bodyCache.set(path, context);
  return context;
}

function deriveJointWorldPoints(body0, body1, linkType0, linkType1, clockwise0, clockwise1) {
  const rolling0 = isRollingLink(linkType0);
  const rolling1 = isRollingLink(linkType1);

  let world0 = body0.position.clone();
  let world1 = body1.position.clone();

  if (rolling0) {
    world0 = deriveRollingSidePoint(body0, body1, linkType0, clockwise0, true);
  }
  if (rolling1) {
    world1 = deriveRollingSidePoint(body1, body0, linkType1, clockwise1, false);
  }

  return { world0, world1 };
}

function resolveJoint(jointPath, jointPrim, context) {
  const {
    body0,
    body1,
    initPolicy,
    linkType0,
    linkType1,
    clockwise0,
    clockwise1,
  } = context;

  const derivedWorld = deriveJointWorldPoints(body0, body1, linkType0, linkType1, clockwise0, clockwise1);
  const authoredLocal0 = readVectorAttribute(jointPrim, 'localPos0');
  const authoredLocal1 = readVectorAttribute(jointPrim, 'localPos1');
  const authoredRestLengthDecl = findDeclaration(jointPrim, 'restLength');
  const authoredRestLength = authoredRestLengthDecl ? Number(authoredRestLengthDecl.value) : null;

  let resolvedLocal0 = null;
  if (initPolicy === DERIVE_ALL_POLICY) {
    resolvedLocal0 = computeLocalPoint(body0.transform, derivedWorld.world0);
  } else if (authoredLocal0) {
    resolvedLocal0 = authoredLocal0;
  } else if (initPolicy === MANUAL_MODE) {
    throw new Error(`${jointPath}: localPos0 is required when cablePath:initPolicy is "manual".`);
  } else {
    resolvedLocal0 = computeLocalPoint(body0.transform, derivedWorld.world0);
  }

  let resolvedLocal1 = null;
  if (initPolicy === DERIVE_ALL_POLICY) {
    resolvedLocal1 = computeLocalPoint(body1.transform, derivedWorld.world1);
  } else if (authoredLocal1) {
    resolvedLocal1 = authoredLocal1;
  } else if (initPolicy === MANUAL_MODE) {
    throw new Error(`${jointPath}: localPos1 is required when cablePath:initPolicy is "manual".`);
  } else {
    resolvedLocal1 = computeLocalPoint(body1.transform, derivedWorld.world1);
  }

  const resolvedWorld0 = computeWorldPoint(body0.transform, resolvedLocal0);
  const resolvedWorld1 = computeWorldPoint(body1.transform, resolvedLocal1);
  const derivedRestLength = resolvedWorld0.distanceTo(resolvedWorld1);

  let resolvedRestLength = null;
  if (initPolicy === DERIVE_ALL_POLICY) {
    resolvedRestLength = derivedRestLength;
  } else if (Number.isFinite(authoredRestLength)) {
    resolvedRestLength = authoredRestLength;
  } else if (initPolicy === MANUAL_MODE) {
    throw new Error(`${jointPath}: restLength is required when cablePath:initPolicy is "manual".`);
  } else {
    resolvedRestLength = derivedRestLength;
  }

  setDeclaration(jointPrim, 'localPos0', fromVector3(resolvedLocal0), 'point3d', 'custom');
  setDeclaration(jointPrim, 'localPos1', fromVector3(resolvedLocal1), 'point3d', 'custom');
  setDeclaration(jointPrim, 'restLength', resolvedRestLength, 'double', 'custom');

  return {
    jointPath,
    body0Path: body0.path,
    body1Path: body1.path,
    world0: resolvedWorld0,
    world1: resolvedWorld1,
    local0: resolvedLocal0,
    local1: resolvedLocal1,
    restLength: resolvedRestLength,
  };
}

function computeAutoStored(linkIndex, linkType, pathContext) {
  if (!isDerivableStoredLink(linkType)) {
    return null;
  }
  if (linkIndex <= 0 || linkIndex >= pathContext.entityPaths.length - 1) {
    return 0.0;
  }

  const leftJoint = pathContext.jointResults[linkIndex - 1];
  const rightJoint = pathContext.jointResults[linkIndex];
  const body = pathContext.bodies[linkIndex];
  const radians = signedArcLengthOnWheel(
    leftJoint.world1,
    rightJoint.world0,
    body.position,
    1.0,
    Boolean(pathContext.clockwise[linkIndex]),
    body.planeNormal,
    true,
  );
  return Math.max(0.0, body.radius + pathContext.halfWidth) * radians;
}

function resolveStoredValue(linkIndex, pathContext) {
  const { initPolicy, linkTypes, storedValues, storedModes, pathPrim } = pathContext;
  const linkType = linkTypes[linkIndex];
  const hasAuthoredStored = Number.isFinite(storedValues[linkIndex]);
  const storedMode = storedModes[linkIndex];
  const autoStored = computeAutoStored(linkIndex, linkType, pathContext);

  if (initPolicy === MANUAL_MODE) {
    if (!hasAuthoredStored) {
      throw new Error(
        `${pathPrim.name}: cablePath:stored[${linkIndex}] is required when cablePath:initPolicy is "manual".`,
      );
    }
    return storedValues[linkIndex];
  }

  if (initPolicy === DERIVE_ALL_POLICY && autoStored !== null) {
    return autoStored;
  }

  if (storedMode === MANUAL_MODE) {
    if (!hasAuthoredStored) {
      throw new Error(
        `${pathPrim.name}: cablePath:stored[${linkIndex}] is required when cablePath:storedMode[${linkIndex}] is "manual".`,
      );
    }
    return storedValues[linkIndex];
  }

  if (storedMode === AUTO_MODE && autoStored !== null) {
    return autoStored;
  }

  if (hasAuthoredStored) {
    return storedValues[linkIndex];
  }

  if (autoStored !== null) {
    return autoStored;
  }

  return 0.0;
}

function resolvePath(pathPrim, pathPath, index, transformCache, bodyCache, seenJoints, options = {}) {
  const jointPaths = getRelationship(pathPrim, 'cablePath:joints');
  const linkTypes = normalizeToPlainArray(getAttribute(pathPrim, 'cablePath:linkTypes')) ?? [];
  const clockwise = (normalizeToPlainArray(getAttribute(pathPrim, 'cablePath:clockwise')) ?? [])
    .map((value) => Boolean(value));

  if (jointPaths.length + 1 !== linkTypes.length) {
    throw new Error(`${pathPath}: cablePath:linkTypes must have joint count + 1 entries.`);
  }
  if (clockwise.length !== linkTypes.length) {
    throw new Error(`${pathPath}: cablePath:clockwise must match cablePath:linkTypes length.`);
  }

  const initPolicy = options.deriveAll === true
    ? DERIVE_ALL_POLICY
    : normalizeInitPolicy(getAttribute(pathPrim, 'cablePath:initPolicy'));
  const storedValues = normalizeStoredValues(pathPrim, linkTypes.length);
  const storedModes = normalizeStoredModes(pathPrim, linkTypes.length);
  const halfWidthOverride = options.cablePathHalfWidthOverride;
  const authoredHalfWidth = Number(getAttribute(pathPrim, 'cablePath:halfWidth') ?? 0.0);
  const halfWidth = Math.max(0.0, Number.isFinite(halfWidthOverride)
    ? halfWidthOverride
    : (Number.isFinite(authoredHalfWidth) ? authoredHalfWidth : 0.0));

  const jointDefs = jointPaths.map((jointPath) => {
    const jointPrim = index[jointPath];
    if (!jointPrim) {
      throw new Error(`${pathPath}: missing CableJoint at ${jointPath}.`);
    }
    if (seenJoints.has(jointPath)) {
      throw new Error(`${jointPath}: referenced by multiple CablePath prims, which is not supported.`);
    }
    seenJoints.add(jointPath);

    const body0Targets = getRelationship(jointPrim, 'physics:body0');
    const body1Targets = getRelationship(jointPrim, 'physics:body1');
    if (body0Targets.length !== 1 || body1Targets.length !== 1) {
      throw new Error(`${jointPath}: CableJoint must have exactly one physics:body0 and physics:body1 target.`);
    }
    return {
      jointPath,
      jointPrim,
      body0Path: body0Targets[0],
      body1Path: body1Targets[0],
    };
  });

  const entityPaths = [];
  for (let indexWithinPath = 0; indexWithinPath < jointDefs.length; indexWithinPath += 1) {
    const jointDef = jointDefs[indexWithinPath];
    if (indexWithinPath === 0) {
      entityPaths.push(jointDef.body0Path);
    } else if (entityPaths[indexWithinPath] !== jointDef.body0Path) {
      throw new Error(
        `${pathPath}: cablePath:joints do not form a continuous body chain at ${jointDef.jointPath}.`,
      );
    }
    entityPaths.push(jointDef.body1Path);
  }

  const bodies = entityPaths.map((bodyPath) => buildBodyContext(index, bodyPath, transformCache, bodyCache));
  const jointResults = jointDefs.map((jointDef, jointIndex) => resolveJoint(
    jointDef.jointPath,
    jointDef.jointPrim,
    {
      body0: bodies[jointIndex],
      body1: bodies[jointIndex + 1],
      initPolicy,
      linkType0: linkTypes[jointIndex],
      linkType1: linkTypes[jointIndex + 1],
      clockwise0: effectiveCW(clockwise, jointIndex, true),
      clockwise1: effectiveCW(clockwise, jointIndex + 1, false),
    },
  ));

  const pathContext = {
    pathPrim,
    initPolicy,
    linkTypes,
    clockwise,
    storedValues,
    storedModes,
    halfWidth,
    entityPaths,
    bodies,
    jointResults,
  };
  const resolvedStored = linkTypes.map((_, linkIndex) => resolveStoredValue(linkIndex, pathContext));

  setDeclaration(pathPrim, 'cablePath:stored', resolvedStored, 'double[]', 'custom');
  setDeclaration(
    pathPrim,
    'cablePath:storedMode',
    resolvedStored.map(() => MANUAL_MODE),
    'token[]',
    'custom',
  );
  setDeclaration(pathPrim, 'cablePath:initPolicy', MANUAL_MODE, 'token', 'custom');
  if (Number.isFinite(halfWidthOverride)) {
    setDeclaration(pathPrim, 'cablePath:halfWidth', halfWidth, 'double', 'custom');
  }

  return {
    pathPath,
    resolvedStored,
    jointResults,
  };
}

export function bakeCableSceneAst(ast, options = {}) {
  const bakedAst = cloneAst(ast);
  const index = indexPrims(bakedAst.statements);
  const transformCache = new Map();
  const bodyCache = new Map();
  const seenJoints = new Set();

  const pathEntries = Object.entries(index)
    .filter(([, prim]) => getRelationship(prim, 'cablePath:joints').length > 0)
    .sort(([left], [right]) => left.localeCompare(right));

  const resolvedPaths = pathEntries.map(([pathPath, pathPrim]) => resolvePath(
    pathPrim,
    pathPath,
    index,
    transformCache,
    bodyCache,
    seenJoints,
    options,
  ));

  return {
    ast: bakedAst,
    resolvedPaths,
  };
}

export function bakeCableSceneUsdaSource(sourceText, options = {}) {
  const parsedAst = parseUsdaAst(sourceText);
  const baked = bakeCableSceneAst(parsedAst, options);
  return {
    ...baked,
    source: serializeUsdaSceneAst(baked.ast),
  };
}
