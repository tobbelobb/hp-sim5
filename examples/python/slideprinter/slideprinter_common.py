from dataclasses import dataclass, field
import math

import sys
from pathlib import Path

root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))

examples_python_path = root_dir / "examples" / "python"
if str(examples_python_path) not in sys.path:
    sys.path.insert(0, str(examples_python_path))

from cable_joints.ecs import (
    OrientationComponent,
    AngularVelocityComponent
)


@dataclass
class SpoolTagComponent:
    """A tag component for entities that are spools."""
    pass

@dataclass
class SpoolStateComponent:
    """Represents the state of a spool mechanism with rotational behavior."""
    axis: str = None
