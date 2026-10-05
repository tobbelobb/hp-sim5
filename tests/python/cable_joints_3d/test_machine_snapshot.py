import copy
import json

import pytest

from parity_harness import FIXTURES, assert_equivalent, run_js, run_python


@pytest.mark.parametrize('name', ['machine_pipeline_minimal', 'machine_pipeline_hp4_commands',
                                 'extruder_rigid_constraints', 'cable_cache_members', 'torque_motor_pinhole',
                                 'cable_construction', 'machine_lifecycle_replace', 'machine_lifecycle_append',
                                 'machine_pipeline_hp4_loaded_torque'])
def test_native_snapshot_matches_the_production_js_flight_recorder(name):
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    fixture['flightSnapshot'] = True
    fields = fixture['tolerance'].get('fields', {})
    if fields:
        for target, source in [('flightSnapshot.frames.position', 'PositionComponent.pos'),
                               ('flightSnapshot.frames.quaternion', 'OrientationComponent.quaternion'),
                               ('flightSnapshot.cables.segments.points', 'PositionComponent.pos'),
                               ('flightSnapshot.cables.segments.origin', 'PositionComponent.pos'),
                               ('flightSnapshot.cables.segments.rest_length', 'CableJointComponent.restLength'),
                               ('flightSnapshot.cables.segments.geometric_length', 'CableJointComponent.geometricLength'),
                               ('flightSnapshot.cables.segments.force_n', 'CableJointComponent.constraintForceMagnitude'),
                               ('flightSnapshot.cables.segments.force_vector_n', 'CableJointComponent.constraintForce'),
                               ('flightSnapshot.cables.lengths', 'CableJointComponent.restLength')]:
            fields[target] = copy.deepcopy(fields[source])
    assert_equivalent(run_python(fixture), run_js(fixture), **fixture['tolerance'], path=name)
