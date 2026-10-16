import pytest

from cable_joints_3d.ecs import World


def test_physics_substeps_keep_command_and_recording_cadence():
    world = World()
    world.set_resource('dt', .002)
    world.set_resource('physicsSubsteps', 4)
    calls = []

    class System:
        def __init__(self, name, physics):
            self.name = name
            self.physics_substep = physics

        def update(self, state, dt):
            calls.append((self.name, dt, state.get_resource('dt')))

    for name, physics in [('command', False), ('integrate', True), ('solve', True), ('record', False)]:
        world.register_system(System(name, physics))
    world.update(.002)
    assert calls == [('command', .002, .002)] + [
        row for _ in range(4) for row in [('integrate', .0005, .0005), ('solve', .0005, .0005)]
    ] + [('record', .002, .002)]
    assert world.get_resource('dt') == .002


def test_physics_substeps_restore_timestep_on_error():
    world = World()
    world.set_resource('dt', .002)
    world.set_resource('physicsSubsteps', 4)

    class BrokenPhysics:
        physics_substep = True

        def update(self, state, dt):
            assert dt == state.get_resource('dt') == .0005
            raise ValueError('solver failed')

    world.register_system(BrokenPhysics())
    with pytest.raises(ValueError, match='solver failed'):
        world.update(.002)
    assert world.get_resource('dt') == .002
