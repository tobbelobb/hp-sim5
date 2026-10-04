"""Rerun renderer for interactive Python simulations and RRD recordings."""
from dataclasses import dataclass, field
import re
from .ecs import (
    MachineTagComponent, RadiusComponent, RenderableComponent,
    OrientationComponent, PositionComponent, SceneEntityInfoComponent,
)

def _safe_path(name): return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "entity"


def _rgb(color):
    value = color.lstrip("#")
    if len(value) == 3:
        value = "".join(character * 2 for character in value)
    return [int(value[index:index + 2], 16) for index in (0, 2, 4)]

@dataclass
class RerunSystem:
    """Log ECS transforms while leaving simulation code independent of the UI."""
    recording: object
    root: str = "world"
    timeline: str = "sim_time"
    elapsed: float = 0.
    _active_paths: set[str] = field(default_factory=set, init=False)
    _shape_styles: dict[str, tuple] = field(default_factory=dict, init=False)
    _entity_tokens: dict[str, object] = field(default_factory=dict, init=False)
    run_in_pause = True

    def _clear_path(self, rr, path):
        # Static archetypes must be cleared on the static timeline. A temporal
        # clear cannot shadow them in Rerun's latest-at queries.
        self.recording.log(
            f"{path}/shape", rr.Clear(recursive=True), static=True
        )
        self.recording.log(path, rr.Clear(recursive=True))

    def update(self, world, dt):
        import rerun as rr

        pause_state = world.get_resource("pauseState")
        if not (pause_state is not None and getattr(pause_state, "paused", False)):
            self.elapsed += dt
        self.recording.set_time(self.timeline, duration=self.elapsed)
        active_paths = set()
        for entity in world.query([PositionComponent]):
            info = world.get_component(entity, SceneEntityInfoComponent)
            machine = world.get_component(entity, MachineTagComponent)
            machine_id = _safe_path(machine.id if machine else "default")
            name = _safe_path(info.name if info else "entity")
            path = f"{self.root}/machines/{machine_id}/{name}_{entity}"
            active_paths.add(path)
            position_component = world.get_component(entity, PositionComponent)
            if (path in self._entity_tokens
                    and self._entity_tokens[path] is not position_component):
                self._clear_path(rr, path)
                self._shape_styles.pop(path, None)
            self._entity_tokens[path] = position_component
            position = position_component.pos
            orientation = world.get_component(entity, OrientationComponent)
            transform = rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=orientation.quaternion.as_xyzw())) if orientation else rr.Transform3D(translation=position)
            self.recording.log(path, transform)
            renderable = world.get_component(entity, RenderableComponent); radius = world.get_component(entity, RadiusComponent)
            if renderable and radius:
                style = (renderable.shape, renderable.color, radius.radius)
                if self._shape_styles.get(path) != style:
                    self.recording.log(
                        f"{path}/shape",
                        rr.Points3D(
                            [[0, 0, 0]],
                            colors=_rgb(renderable.color),
                            radii=radius.radius,
                        ),
                        static=True,
                    )
                    self._shape_styles[path] = style

        for path in self._active_paths - active_paths:
            self._clear_path(rr, path)
            self._shape_styles.pop(path, None)
            self._entity_tokens.pop(path, None)
        self._active_paths = active_paths
