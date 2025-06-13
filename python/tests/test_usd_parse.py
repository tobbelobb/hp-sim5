import numpy as np
from slideprinter_usd_demo import parse_slideprinter


def test_parse_slideprinter_basic():
    model, entities, joints, paths, _ = parse_slideprinter('slideprinter/slideprinter.usda')
    spools = [e for e in entities.values() if e['type'] == 'spool']
    anchors = [e for e in entities.values() if e['type'] == 'anchor']
    assert len(spools) == 3
    assert len(anchors) == 3
    assert model is None or model.body_count >= 0


def test_parse_flipper_scene_joints_and_paths():
    model, entities, joints, paths, _ = parse_slideprinter('flipper_scene.usda')
    names = set(entities.keys())
    assert {'Ball1', 'Ball2', 'Obs3', 'Obs4'}.issubset(names)
    assert len(joints) == 3
    assert len(paths) == 1
    assert paths[0]['linkTypes'][0] == 'hybrid-attachment'
