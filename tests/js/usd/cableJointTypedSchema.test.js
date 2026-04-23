const fs = require('node:fs');
const path = require('node:path');

describe('CablePathAPI schema', () => {
  test('matches the authored cablePath property names', () => {
    const schemaPath = path.resolve(
      __dirname,
      '../../../public/usd_scenes/cable_joint_typed_schema/schema.usda'
    );
    const schemaText = fs.readFileSync(schemaPath, 'utf8');

    expect(schemaText).toContain('rel cablePath:joints[]');
    expect(schemaText).toContain('token[] cablePath:linkTypes');
    expect(schemaText).toContain('bool[] cablePath:clockwise');
    expect(schemaText).toContain('double[] cablePath:stored');
    expect(schemaText).toContain('double cablePath:totalLength');
    expect(schemaText).toContain('double stiffness = 1000.0');
    expect(schemaText).toContain('double cablePath:damping = 0.0');
    expect(schemaText).toContain('double cablePath:halfWidth = 0.0');
    expect(schemaText).toContain('bool cablePath:enabled = true');
    expect(schemaText).toContain('double cablePath:tension = 0.0');

    expect(schemaText).not.toContain('rel cableJoints[]');
    expect(schemaText).not.toContain('double[] storedLength');
  });
});
