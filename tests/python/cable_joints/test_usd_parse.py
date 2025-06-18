import numpy as np
from slideprinter_usd_demo import parse_slideprinter


def test_parse_slideprinter_basic():
    model, entities, joints, paths, _ = parse_slideprinter('examples/usd_scenes/slideprinter.usda')
    spools = [e for e in entities.values() if e['type'] == 'spool']
    anchors = [e for e in entities.values() if e['type'] == 'anchor']
    assert len(spools) == 3
    assert len(anchors) == 3
    assert model is None or model.body_count >= 0
