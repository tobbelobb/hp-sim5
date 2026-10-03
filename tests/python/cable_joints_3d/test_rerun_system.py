import sys
from types import SimpleNamespace

from cable_joints.ecs import RadiusComponent, RenderableComponent, World
from cable_joints_3d.ecs import PositionComponent, SceneEntityInfoComponent
from cable_joints_3d.rerun_system import RerunSystem


class Recording:
    def __init__(self):
        self.times = []
        self.logs = []

    def set_time(self, timeline, duration):
        self.times.append((timeline, duration))

    def log(self, path, value, **kwargs):
        self.logs.append((path, value, kwargs))


def _fake_rerun():
    return SimpleNamespace(
        Transform3D=lambda **kwargs: ("transform", kwargs),
        Quaternion=lambda **kwargs: ("quaternion", kwargs),
        Points3D=lambda *args, **kwargs: ("points", args, kwargs),
        Clear=lambda **kwargs: ("clear", kwargs),
    )


def test_rerun_time_static_styles_and_removed_entities(monkeypatch):
    monkeypatch.setitem(sys.modules, "rerun", _fake_rerun())
    world = World(); world.set_resource("pauseState", SimpleNamespace(paused=False))
    entity = world.create_entity()
    world.add_component(entity, PositionComponent(1, 2, 3))
    world.add_component(entity, SceneEntityInfoComponent("test ball"))
    world.add_component(entity, RadiusComponent(.2))
    world.add_component(entity, RenderableComponent("circle", "#1a2B3c"))
    recording = Recording(); system = RerunSystem(recording)

    system.update(world, .1)
    world.get_resource("pauseState").paused = True
    system.update(world, .1)

    assert recording.times == [("sim_time", .1), ("sim_time", .1)]
    shapes = [log for log in recording.logs if log[0].endswith("/shape")]
    assert len(shapes) == 1
    assert shapes[0][1][2]["colors"] == [26, 43, 60]

    world.destroy_entity(entity)
    system.update(world, .1)
    assert recording.logs[-1] == (
        "world/entities/test_ball", ("clear", {"recursive": True}), {}
    )


def test_rerun_clears_reused_path_during_scene_reset(monkeypatch):
    monkeypatch.setitem(sys.modules, "rerun", _fake_rerun())
    world = World(); recording = Recording(); system = RerunSystem(recording)
    old_entity = world.create_entity()
    world.add_component(old_entity, PositionComponent())
    world.add_component(old_entity, SceneEntityInfoComponent("effector"))
    world.add_component(old_entity, RadiusComponent(.2))
    world.add_component(old_entity, RenderableComponent("circle", "#ffffff"))
    system.update(world, .1)

    world.clear()
    new_entity = world.create_entity()
    world.add_component(new_entity, PositionComponent(2, 0, 0))
    world.add_component(new_entity, SceneEntityInfoComponent("effector"))
    system.update(world, .1)

    clears = [value for path, value, _ in recording.logs
              if path == "world/entities/effector" and value[0] == "clear"]
    assert len(clears) == 1
