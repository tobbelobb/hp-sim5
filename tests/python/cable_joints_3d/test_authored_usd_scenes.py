from pxr import Sdf, Usd

from parity_harness import ROOT


def test_cube_scene_authors_euler_rotation_with_a_three_vector_type():
    stage = Usd.Stage.Open(str(ROOT / 'public/usd_scenes/cubecorners_rigid_body.usda'))
    rotation = stage.GetPrimAtPath('/World/HangprinterScene/SpoolA').GetAttribute('xformOp:rotateXYZ')
    assert rotation.GetTypeName() == Sdf.ValueTypeNames.Double3
    assert tuple(rotation.Get()) == (0, 0, 20)
