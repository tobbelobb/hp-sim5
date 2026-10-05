import { OpenText, getAttribute, getRelationship } from '../../../src/js/usd/stage.js';

describe('USD authored attribute precision', () => {
  test.each(['float', 'float2', 'float3', 'float4', 'point3f', 'vector3f',
    'normal3f', 'color3f', 'color4f', 'texCoord2f', 'texCoord3f', 'quatf'])(
    '%s values retain their USD single-precision opinions', type => {
      const arity = type === 'float' ? 1 : /[24]/.test(type) ? Number(type.match(/[24]/)[0]) : type === 'quatf' ? 4 : 3;
      const literal = arity === 1 ? '0.1' : `(${['0.1', '0.2', '-0.3', '0.8'].slice(0, arity).join(', ')})`;
      const stage = OpenText(`#usda 1.0
def Xform "Prim" {
    custom ${type} value = ${literal}
    custom ${type}[] values = [${literal}, ${literal}]
}`);
      // Fixed IEEE binary32 values, independent of the loader implementation.
      const rounded = [0.10000000149011612, 0.20000000298023224,
        -0.30000001192092896, 0.800000011920929];
      const expected = arity === 1 ? rounded[0] : rounded.slice(0, arity);
      const prim = stage.GetPrimAtPath('/Prim');
      expect(getAttribute(prim, 'value')).toEqual(expected);
      expect(getAttribute(prim, 'values')).toEqual([expected, expected]);
      expect(prim.statements.find(statement => statement.reference === 'value').value).toEqual(expected);
    });

  test('double values, metadata and relationships retain their values', () => {
    const stage = OpenText(`#usda 1.0
def Xform "Prim" (apiSchemas = ["CablePathAPI"]) {
    custom double value = 0.1
    custom double3 points = (0.1, 0.2, -0.3)
    custom double[] values = [0.1, 0.2]
    custom token label = "float3"
    custom float unauthored
    custom rel target = </Prim/Child>
    def Xform "Child" { custom float value = 0.2 }
}`);
    const prim = stage.GetPrimAtPath('/Prim');
    expect(getAttribute(prim, 'value')).toBe(0.1);
    expect(getAttribute(prim, 'points')).toEqual([0.1, 0.2, -0.3]);
    expect(getAttribute(prim, 'values')).toEqual([0.1, 0.2]);
    expect(getAttribute(prim, 'label')).toBe('float3');
    expect(getAttribute(prim, 'unauthored')).toBeNull();
    expect(getAttribute(prim, 'apiSchemas')).toEqual(['CablePathAPI']);
    expect(getRelationship(prim, 'target')).toEqual(['/Prim/Child']);
    expect(getAttribute(stage.GetPrimAtPath('/Prim/Child'), 'value')).toBe(0.20000000298023224);
  });
});
