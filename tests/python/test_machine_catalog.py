import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import machines


def test_catalogued_files_exist():
    for scene in machines.CATALOG['scenes']:
        assert (ROOT / scene).is_file(), scene
    for entry in machines.CATALOG['machines']:
        paths = [entry['scene'], entry['rrf_config'], entry['klipper_config']]
        for gcode, compiled in entry['compiled_commands'].items():
            paths += [gcode, *compiled.values()]
        for path in paths:
            assert (ROOT / path).is_file(), f'{entry["name"]}: {path}'


def test_names_and_nicknames_are_unique():
    names = [name.lower() for entry in machines.CATALOG['machines'] for name in [entry['name'], *entry['nicknames']]]
    assert len(names) == len(set(names))


def test_scene_identity_resolves_to_its_catalogue_defaults():
    import re
    for path, settings in machines.CATALOG['scenes'].items():
        identity = re.search(r'custom string machine:catalog = "([^"]+)"', (ROOT / path).read_text())
        assert identity, path
        assert machines.machine(identity[1])['scene'] == path
        count = settings.get('physics_substeps', 1)
        assert isinstance(count, int) and 1 <= count <= 16
        count = settings.get('solver_iterations', 1)
        assert isinstance(count, int) and 1 <= count <= 64


def test_catalogued_collector_types_are_accepted_by_the_dataset_schema():
    schema = json.loads((ROOT / 'autocal/schemas/sweep_dataset.schema.json').read_text())
    supported = set(schema['properties']['machine_type']['enum'])
    assert {scene['machine_type'] for scene in machines.CATALOG['scenes'].values()
            if 'machine_type' in scene} <= supported


def test_each_scene_has_one_machine_per_line_layering_choice():
    for scene in machines.CATALOG['scenes']:
        variants = sorted(entry['line_layering'] for entry in machines.CATALOG['machines'] if entry['scene'] == scene)
        assert variants == [False, True], scene
    assert {entry['scene'] for entry in machines.CATALOG['machines']} == set(machines.CATALOG['scenes'])


def test_lookup_is_case_insensitive_and_switches_line_layering():
    hp3 = machines.machine('HP3')
    assert hp3['name'] == 'Hangprinter version 3 without line layering'
    assert machines.machine(hp3['name'].upper()) is hp3
    layered = machines.line_layering_variant(hp3, True)
    assert layered['scene'] == hp3['scene'] and layered['rrf_config'] == 'RRF/run/vsd/sys/config_hp3_w_line_layers.g'
    assert machines.machine('skycam')['scene'] == 'public/usd_scenes/four_high_anchors.usda'
    with pytest.raises(ValueError, match='Unknown machine'):
        machines.machine('hp9')
