import numpy as np
from dataclasses import dataclass, field
from typing import List

@dataclass
class CableJointComponent:
    """
    Represents a single segment constraint between two entities.
    Corresponds to CableJointComponent in JavaScript.
    """
    entity_a: int
    entity_b: int
    rest_length: float
    # These are numpy arrays. The caller is responsible for passing copies
    # if the original arrays should not be modified.
    attachment_point_a_world: np.ndarray
    attachment_point_b_world: np.ndarray

@dataclass
class CablePathComponent:
    """
    Connects individual cable joints into a cable path.
    Corresponds to CablePathComponent in JavaScript.

    NOTE: The original JavaScript constructor contained complex logic that
    depended on the `world` object to calculate initial `total_rest_length`
    and `stored` values. In a Python ECS, this logic is better placed in a
    factory function or a system that runs once upon creation, rather than
    in the component's `__init__`. This class is defined as a pure data
    container. The user is responsible for porting and running the
    initialization logic from the original JS constructor.
    """
    joint_entities: List[int] = field(default_factory=list)
    link_types: List[str] = field(default_factory=list)
    cw: List[bool] = field(default_factory=list)
    spring_constant: float = 1e6
    stored: List[float] = field(default_factory=list)
    total_rest_length: float = 0.0
    compliance: float = field(init=False)

    def __post_init__(self):
        self.compliance = 1.0 / self.spring_constant
