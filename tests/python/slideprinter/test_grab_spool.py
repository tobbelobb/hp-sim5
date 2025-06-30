import sys
from pathlib import Path
import numpy as np

# Setup paths
root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))

examples_python_path = root_dir / "examples" / "python"
if str(examples_python_path) not in sys.path:
    sys.path.insert(0, str(examples_python_path))

from cable_joints.ecs import World, PositionComponent
from cable_joints.cable_joints_components import CableJointComponent, CablePathComponent
from slideprinter.server import setup_scene, GrabSpoolSystem
from slideprinter.slideprinter_common import SpoolTagComponent

def test_grab_move_and_release_spool():
    """Tests grabbing, moving, and releasing a spool."""
    # 1. Setup
    world = World()
    setup_scene(world)
    grab_system = world.get_system(GrabSpoolSystem)
    assert grab_system is not None
    assert grab_system._grab_spring is None

    # Find a spool to grab
    spool_id = -1
    spool_pos = None
    for e in world.query([SpoolTagComponent, PositionComponent]):
        spool_id = e
        spool_pos = world.get_component(e, PositionComponent).pos
        break
    
    assert spool_id != -1
    assert spool_pos is not None

    # 2. Grab the spool
    grab_system.add_event({'type': 'pointerdown', 'pos': spool_pos.copy()})
    world.update(0.01)

    # 3. Assertions for grab
    assert grab_system._grab_spring is not None
    ptr_e, joint_e, path_e, grabbed_spool_e = grab_system._grab_spring
    assert grabbed_spool_e == spool_id
    assert world.has_component(ptr_e, PositionComponent)
    assert world.has_component(joint_e, CableJointComponent)
    assert world.has_component(path_e, CablePathComponent)
    assert world.get_resource('grabbedBall') == spool_id

    # 4. Move the pointer
    new_pos = spool_pos + np.array([0.1, 0.1, 0.0])
    grab_system.add_event({'type': 'pointermove', 'pos': new_pos.copy()})
    world.update(0.01)

    # 5. Assertions for move
    ptr_pos_comp = world.get_component(ptr_e, PositionComponent)
    assert np.allclose(ptr_pos_comp.pos, new_pos)

    # 6. Release the spool
    grab_system.add_event({'type': 'pointerup', 'pos': new_pos.copy()})
    world.update(0.01)

    # 7. Assertions for release
    assert grab_system._grab_spring is None
    assert not world.has_component(ptr_e, PositionComponent)
    assert not world.has_component(joint_e, CableJointComponent)
    assert not world.has_component(path_e, CablePathComponent)
    assert world.get_resource('grabbedBall') is None

def test_grab_nothing_on_empty_space_click():
    """Tests that clicking on empty space doesn't create a grab spring."""
    # 1. Setup
    world = World()
    setup_scene(world)
    grab_system = world.get_system(GrabSpoolSystem)
    assert grab_system is not None
    assert grab_system._grab_spring is None

    # 2. Click somewhere far from any spool
    empty_space_pos = np.array([-10.0, -10.0, 0.0])
    grab_system.add_event({'type': 'pointerdown', 'pos': empty_space_pos})
    world.update(0.01)

    # 3. Assertions
    assert grab_system._grab_spring is None
    assert world.get_resource('grabbedBall') is None
