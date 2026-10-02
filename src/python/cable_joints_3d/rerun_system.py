"""Rerun renderer for interactive Python simulations and RRD recordings."""
from dataclasses import dataclass
import re
from cable_joints.ecs import RadiusComponent, RenderableComponent
from .ecs import OrientationComponent, PositionComponent, SceneEntityInfoComponent

def _safe_path(name): return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_") or "entity"

@dataclass
class RerunSystem:
    """Log ECS transforms while leaving simulation code independent of the UI."""
    recording: object
    root: str = "world/entities"
    timeline: str = "sim_time"
    elapsed: float = 0.
    run_in_pause = True
    def update(self, world, dt):
        import rerun as rr
        self.elapsed += dt; self.recording.set_time(self.timeline, duration=self.elapsed)
        for entity in world.query([PositionComponent]):
            info = world.get_component(entity, SceneEntityInfoComponent)
            path = f"{self.root}/{_safe_path(info.name if info else str(entity))}"
            position = world.get_component(entity, PositionComponent).pos
            orientation = world.get_component(entity, OrientationComponent)
            transform = rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=orientation.quaternion.as_xyzw())) if orientation else rr.Transform3D(translation=position)
            self.recording.log(path, transform)
            renderable = world.get_component(entity, RenderableComponent); radius = world.get_component(entity, RadiusComponent)
            if renderable and radius:
                self.recording.log(f"{path}/shape", rr.Points3D([[0, 0, 0]], colors=renderable.color, radii=radius.radius), static=True)
