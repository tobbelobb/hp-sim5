from dataclasses import dataclass, field
import math

@dataclass
class SpoolTagComponent:
    """A tag component for entities that are spools."""
    pass

@dataclass
class SpoolStateComponent:
    """Represents the state of a spool mechanism with rotational behavior."""
    target_angle: float = 0.0
    current_angle: float = 0.0
    speed: float = 0.0  # rad/min

class SlideprinterMotionSystem:
    def update(self, world, dt):
        for entity_id in world.query([SpoolStateComponent, SpoolTagComponent]):
            state = world.get_component(entity_id, SpoolStateComponent)
            
            delta_angle = state.target_angle - state.current_angle
            if abs(delta_angle) < 1e-6:
                continue

            speed_rad_per_sec = state.speed / 60.0
            max_rotation_this_frame = speed_rad_per_sec * dt
            
            if abs(delta_angle) < max_rotation_this_frame:
                state.current_angle = state.target_angle
            else:
                state.current_angle += math.copysign(max_rotation_this_frame, delta_angle)
