import sys
from types import SimpleNamespace

from cable_joints_3d.ecs import (
    MachineTagComponent, RadiusComponent, RenderableComponent, World,
    PositionComponent, SceneEntityInfoComponent,
)
from cable_joints_3d.rerun_system import RerunSystem


class Recording:
    def __init__(self):
        self.times = []
        self.logs = []
        self.static = {}

    def set_time(self, timeline, **values):
        self.times.append((timeline, values))

    def send_columns(self, path, *, indexes, columns, **kwargs):
        name, fields, lengths = columns
        start = 0
        for length in lengths:
            row = {key: value[start:start + length] for key, value in fields.items()}
            if name == 'transform':
                value = (name, {'translation': row['translation'][0],
                                'rotation': ('quaternion', {'xyzw': row['quaternion'][0]})})
            else:
                value = (name, row)
            self.log(path, value)
            start += length

    def log(self, path, value, **kwargs):
        self.logs.append((path, value, kwargs))
        if kwargs.get("static"):
            if value[0] == "clear":
                for stored_path in list(self.static):
                    if stored_path == path or stored_path.startswith(f"{path}/"):
                        del self.static[stored_path]
            else:
                self.static[path] = value


class Columns:
    def __init__(self, name, fields):
        self.name, self.fields = name, fields

    def partition(self, lengths):
        return self.name, self.fields, lengths


def _fake_rerun():
    def archetype(name, constructor):
        constructor.columns = lambda **fields: Columns(name, fields)
        return constructor

    return SimpleNamespace(
        Transform3D=archetype("transform", lambda **kwargs: ("transform", kwargs)),
        TransformAxes3D=lambda value: ("axes", value),
        Quaternion=lambda **kwargs: ("quaternion", kwargs),
        Points3D=archetype("points", lambda *args, **kwargs: ("points", args, kwargs)),
        Clear=lambda **kwargs: ("clear", kwargs),
        LineStrips3D=archetype("lines", lambda *args, **kwargs: ("lines", args, kwargs)),
        Arrows3D=archetype("arrows", lambda **kwargs: ("arrows", kwargs)),
        SeriesLines=lambda **kwargs: ("series", kwargs),
        Scalars=archetype("scalars", lambda values: ("scalars", values)),
        Radius=SimpleNamespace(ui_points=lambda value: value),
        ViewCoordinates=SimpleNamespace(RIGHT_HAND_Z_UP=("coordinates", "z_up")),
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

    assert recording.times == [("scene_generation", {"sequence": 0}), ("sim_step", {"sequence": 1}),
                               ("sim_time", {"duration": .1})] * 2
    shapes = [log for log in recording.logs if log[0].endswith("/shape")]
    assert len(shapes) == 1
    assert shapes[0][1][2]["colors"] == [26, 43, 60]

    world.destroy_entity(entity)
    system.update(world, .1)
    assert recording.logs[-1] == (
        "world/machines/default/bodies/test_ball_0",
        ("clear", {"recursive": True}), {},
    )
    assert set(recording.static) == {"world"}


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

    shape_path = "world/machines/default/bodies/effector_0/shape"
    static_clears = [value for path, value, kwargs in recording.logs
                     if path == shape_path[:-6] and value[0] == "clear"
                     and kwargs.get("static")]
    assert len(static_clears) == 1
    assert shape_path not in recording.static


def test_rerun_paths_include_machine_and_entity_identity(monkeypatch):
    monkeypatch.setitem(sys.modules, "rerun", _fake_rerun())
    world = World(); recording = Recording(); system = RerunSystem(recording)
    for machine_id in ("left", "right"):
        entity = world.create_entity()
        world.add_component(entity, PositionComponent())
        world.add_component(entity, SceneEntityInfoComponent("anchor"))
        world.add_component(entity, MachineTagComponent(machine_id))
    duplicate = world.create_entity()
    world.add_component(duplicate, PositionComponent())
    world.add_component(duplicate, SceneEntityInfoComponent("anchor"))
    world.add_component(duplicate, MachineTagComponent("left"))

    system.update(world, .1)

    system.flush()
    transform_paths = [path for path, value, _ in recording.logs
                       if value[0] == "transform"]
    assert transform_paths == [
        "world/machines/left/bodies/anchor_0",
        "world/machines/right/bodies/anchor_1",
        "world/machines/left/bodies/anchor_2",
    ]
