const LENGTH_ATTRIBUTES = new Set([
  'radius',
  'height',
  'physics:gravityMagnitude',
  'physics:velocity',
  'physics:minDistance',
  'physics:maxDistance',
  'localPos0',
  'localPos1',
  'restLength',
  'cablePath:stored',
  'cablePath:totalLength',
  'cablePath:halfWidth',
]);

function scaleNumbers(value, factor) {
  if (Array.isArray(value)) {
    return value.map(entry => scaleNumbers(entry, factor));
  }
  return typeof value === 'number' ? value * factor : value;
}

function scaleDeclarations(statements, metersPerUnit) {
  for (const statement of statements ?? []) {
    if (statement.type === 'definition') {
      scaleDeclarations(statement.statements, metersPerUnit);
      continue;
    }
    if (statement.type !== 'declaration') continue;

    const name = statement.reference;
    if (LENGTH_ATTRIBUTES.has(name) || /^xformOp:translate(?::|$)/.test(name)) {
      statement.value = scaleNumbers(statement.value, metersPerUnit);
    } else if (name === 'physics:inertiaTensor') {
      statement.value = scaleNumbers(statement.value, metersPerUnit ** 2);
    }
  }
}

// Normalize the machine schema before any cable derivation or entity construction.
// Cable EA and damping are in N and N·s. Stepper and bearing parameters also use SI.
export function normalizeSceneUnits(ast) {
  const units = ast.descriptor?.assignments?.find(
    assignment => assignment.type === 'assignment' && assignment.identifier === 'metersPerUnit',
  );
  // Existing machine scenes without metadata use metres.
  if (!units || units.value === 1) return;
  if (!Number.isFinite(units.value) || units.value <= 0) {
    throw new Error('metersPerUnit must be a finite positive number.');
  }

  scaleDeclarations(ast.statements, units.value);
  units.value = 1;
}
