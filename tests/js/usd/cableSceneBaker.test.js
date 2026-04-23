import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  signedArcLengthOnWheel,
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
} from '../../../src/js/cable_joints_3d/geometry3.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { Open as UsdOpen, getAttribute } from '../../../src/js/usd/stage.js';

const PLANE_NORMAL = new Vector3(0.0, 0.0, 1.0);

function expectVectorClose(actual, expected) {
  expect(actual[0]).toBeCloseTo(expected.x);
  expect(actual[1]).toBeCloseTo(expected.y);
  expect(actual[2] ?? 0.0).toBeCloseTo(expected.z);
}

describe('cable scene baker', () => {
  test('derives missing CableJoint values and rewrites path policy to manual', async () => {
    const source = `#usda 1.0

def Xform "World"
{
    def Xform "Scene"
    {
        def Circle "SpoolA"
        {
            custom bool cable:linkable = 1
            double radius = 1
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "AnchorA"
        {
            double3 xformOp:translate = (3, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def CableJoint "JointA"
        {
            custom rel physics:body0 = </World/Scene/SpoolA>
            custom rel physics:body1 = </World/Scene/AnchorA>
        }

        def Xform "CablePathA" (
            apiSchemas = ["CablePathAPI"]
        )
        {
            custom bool[] cablePath:clockwise = [1, 1]
            custom rel cablePath:joints = [</World/Scene/JointA>]
            custom token[] cablePath:linkTypes = ["hybrid", "attachment"]
            custom double cablePath:halfWidth = 0.25
        }
    }
}
`;

    const baked = bakeCableSceneUsdaSource(source);
    const stage = await UsdOpen(baked.source);
    const jointPrim = stage.GetPrimAtPath('/World/Scene/JointA');
    const pathPrim = stage.GetPrimAtPath('/World/Scene/CablePathA');

    const expectedTangent = tangentFromSphereToPoint(
      new Vector3(3.0, 0.0, 0.0),
      new Vector3(0.0, 0.0, 0.0),
      1.0,
      PLANE_NORMAL,
      false,
    ).a_sphere;

    expectVectorClose(getAttribute(jointPrim, 'localPos0'), expectedTangent);
    expectVectorClose(getAttribute(jointPrim, 'localPos1'), new Vector3(0.0, 0.0, 0.0));
    expect(getAttribute(jointPrim, 'restLength')).toBeCloseTo(expectedTangent.distanceTo(new Vector3(3.0, 0.0, 0.0)));

    expect(getAttribute(pathPrim, 'cablePath:initPolicy')).toBe('manual');
    expect(getAttribute(pathPrim, 'cablePath:storedMode')).toEqual(['manual', 'manual']);
    expect(getAttribute(pathPrim, 'cablePath:stored')).toEqual([0.0, 0.0]);
  });

  test('derives only stored entries marked auto under deriveMissing', async () => {
    const source = `#usda 1.0

def Xform "World"
{
    def Xform "Scene"
    {
        def Xform "AnchorL"
        {
            double3 xformOp:translate = (-2, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Circle "Wheel"
        {
            custom bool cable:linkable = 1
            double radius = 1
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "AnchorR"
        {
            double3 xformOp:translate = (2, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def CableJoint "JointL"
        {
            custom rel physics:body0 = </World/Scene/AnchorL>
            custom rel physics:body1 = </World/Scene/Wheel>
        }

        def CableJoint "JointR"
        {
            custom rel physics:body0 = </World/Scene/Wheel>
            custom rel physics:body1 = </World/Scene/AnchorR>
        }

        def Xform "CablePathA" (
            apiSchemas = ["CablePathAPI"]
        )
        {
            custom bool[] cablePath:clockwise = [1, 1, 1]
            custom rel cablePath:joints = [</World/Scene/JointL>, </World/Scene/JointR>]
            custom token[] cablePath:linkTypes = ["attachment", "rolling", "attachment"]
            custom double[] cablePath:stored = [5, 999, 7]
            custom token[] cablePath:storedMode = ["manual", "auto", "manual"]
            custom token cablePath:initPolicy = "deriveMissing"
            custom double cablePath:halfWidth = 0.25
        }
    }
}
`;

    const baked = bakeCableSceneUsdaSource(source);
    const stage = await UsdOpen(baked.source);
    const pathPrim = stage.GetPrimAtPath('/World/Scene/CablePathA');
    const jointLPrim = stage.GetPrimAtPath('/World/Scene/JointL');
    const jointRPrim = stage.GetPrimAtPath('/World/Scene/JointR');

    const leftTangent = tangentFromPointToSphere(
      new Vector3(-2.0, 0.0, 0.0),
      new Vector3(0.0, 0.0, 0.0),
      1.0,
      PLANE_NORMAL,
      true,
    ).a_sphere;
    const rightTangent = tangentFromSphereToPoint(
      new Vector3(2.0, 0.0, 0.0),
      new Vector3(0.0, 0.0, 0.0),
      1.0,
      PLANE_NORMAL,
      true,
    ).a_sphere;
    const expectedStored = 1.25 * signedArcLengthOnWheel(
      leftTangent,
      rightTangent,
      new Vector3(0.0, 0.0, 0.0),
      1.0,
      true,
      PLANE_NORMAL,
      true,
    );

    expectVectorClose(getAttribute(jointLPrim, 'localPos1'), leftTangent);
    expectVectorClose(getAttribute(jointRPrim, 'localPos0'), rightTangent);
    const stored = getAttribute(pathPrim, 'cablePath:stored');
    expect(stored[0]).toBe(5);
    expect(stored[1]).toBeCloseTo(expectedStored);
    expect(stored[2]).toBe(7);
    expect(getAttribute(pathPrim, 'cablePath:initPolicy')).toBe('manual');
    expect(getAttribute(pathPrim, 'cablePath:storedMode')).toEqual(['manual', 'manual', 'manual']);
  });

  test('can override authored cablePath:halfWidth while baking', async () => {
    const source = `#usda 1.0

def Xform "World"
{
    def Xform "Scene"
    {
        def Xform "AnchorL"
        {
            double3 xformOp:translate = (-2, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Circle "Wheel"
        {
            double radius = 1
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "AnchorR"
        {
            double3 xformOp:translate = (2, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def CableJoint "JointL"
        {
            custom rel physics:body0 = </World/Scene/AnchorL>
            custom rel physics:body1 = </World/Scene/Wheel>
        }

        def CableJoint "JointR"
        {
            custom rel physics:body0 = </World/Scene/Wheel>
            custom rel physics:body1 = </World/Scene/AnchorR>
        }

        def Xform "CablePathA" (
            apiSchemas = ["CablePathAPI"]
        )
        {
            custom bool[] cablePath:clockwise = [1, 1, 1]
            custom rel cablePath:joints = [</World/Scene/JointL>, </World/Scene/JointR>]
            custom token[] cablePath:linkTypes = ["attachment", "rolling", "attachment"]
            custom token[] cablePath:storedMode = ["manual", "auto", "manual"]
            custom double[] cablePath:stored = [0, 0, 0]
            custom double cablePath:halfWidth = 0.25
        }
    }
}
`;

    const defaultBaked = bakeCableSceneUsdaSource(source);
    const overrideBaked = bakeCableSceneUsdaSource(source, { cablePathHalfWidthOverride: 0.0 });
    const defaultStage = await UsdOpen(defaultBaked.source);
    const overrideStage = await UsdOpen(overrideBaked.source);
    const defaultPath = defaultStage.GetPrimAtPath('/World/Scene/CablePathA');
    const overridePath = overrideStage.GetPrimAtPath('/World/Scene/CablePathA');

    expect(getAttribute(overridePath, 'cablePath:halfWidth')).toBe(0.0);
    expect(getAttribute(overridePath, 'cablePath:stored')[1]).toBeLessThan(
      getAttribute(defaultPath, 'cablePath:stored')[1]
    );
  });

  test('deriveAll overwrites authored joint values and derivable stored values', async () => {
    const source = `#usda 1.0

def Xform "World"
{
    def Xform "Scene"
    {
        def Xform "AnchorL"
        {
            double3 xformOp:translate = (-2, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Circle "Wheel"
        {
            custom bool cable:linkable = 1
            double radius = 1
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "AnchorR"
        {
            double3 xformOp:translate = (2, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def CableJoint "JointL"
        {
            custom point3d localPos0 = (9, 9, 9)
            custom point3d localPos1 = (8, 8, 8)
            custom rel physics:body0 = </World/Scene/AnchorL>
            custom rel physics:body1 = </World/Scene/Wheel>
            custom double restLength = 123
        }

        def CableJoint "JointR"
        {
            custom point3d localPos0 = (7, 7, 7)
            custom point3d localPos1 = (6, 6, 6)
            custom rel physics:body0 = </World/Scene/Wheel>
            custom rel physics:body1 = </World/Scene/AnchorR>
            custom double restLength = 456
        }

        def Xform "CablePathA" (
            apiSchemas = ["CablePathAPI"]
        )
        {
            custom bool[] cablePath:clockwise = [1, 1, 1]
            custom rel cablePath:joints = [</World/Scene/JointL>, </World/Scene/JointR>]
            custom token[] cablePath:linkTypes = ["attachment", "rolling", "attachment"]
            custom double[] cablePath:stored = [5, 999, 7]
            custom token cablePath:initPolicy = "deriveAll"
            custom double cablePath:halfWidth = 0.25
        }
    }
}
`;

    const baked = bakeCableSceneUsdaSource(source);
    const stage = await UsdOpen(baked.source);
    const jointLPrim = stage.GetPrimAtPath('/World/Scene/JointL');
    const pathPrim = stage.GetPrimAtPath('/World/Scene/CablePathA');

    expect(getAttribute(jointLPrim, 'localPos0')).not.toEqual([9, 9, 9]);
    expect(getAttribute(jointLPrim, 'restLength')).not.toBe(123);
    expect(getAttribute(pathPrim, 'cablePath:stored')[0]).toBe(5);
    expect(getAttribute(pathPrim, 'cablePath:stored')[1]).not.toBe(999);
    expect(getAttribute(pathPrim, 'cablePath:stored')[2]).toBe(7);
  });

  test('manual init policy rejects missing authored values', () => {
    const source = `#usda 1.0

def Xform "World"
{
    def Xform "Scene"
    {
        def Xform "AnchorL"
        {
            double3 xformOp:translate = (-2, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "AnchorR"
        {
            double3 xformOp:translate = (2, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def CableJoint "JointA"
        {
            custom rel physics:body0 = </World/Scene/AnchorL>
            custom rel physics:body1 = </World/Scene/AnchorR>
        }

        def Xform "CablePathA" (
            apiSchemas = ["CablePathAPI"]
        )
        {
            custom bool[] cablePath:clockwise = [1, 1]
            custom rel cablePath:joints = [</World/Scene/JointA>]
            custom token[] cablePath:linkTypes = ["attachment", "attachment"]
            custom token cablePath:initPolicy = "manual"
            custom double[] cablePath:stored = [0, 0]
        }
    }
}
`;

    expect(() => bakeCableSceneUsdaSource(source)).toThrow(/manual/);
  });
});
