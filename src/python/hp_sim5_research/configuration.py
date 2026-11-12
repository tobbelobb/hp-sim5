"""Validated, immutable settings for one coordinated world generation."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

import machines

from .experiments import encode, repo_path

BACKENDS = ('headless-js',)
LAYERED_BUILDUP = 2 / math.pi


def number(name, value, low, high, *, integer=False):
    if (type(value) not in ((int,) if integer else (int, float)) or
            not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f'{name} must be {"an integer" if integer else "a number"} in [{low}, {high}]')
    return value


def validate_spools(layering, radii, buildup):
    if not isinstance(radii, list) or len(radii) not in (1, 4):
        raise ValueError('base_radii_mm must contain one or four radii')
    for radius in radii:
        number('base_radii_mm', radius, 1, 200)
    number('buildup_factor', buildup, 0, 2)
    if not layering and buildup != 0:
        raise ValueError('Nonzero or fitted buildup requires layering')


def firmware_path(machine):
    return str(Path(machine['rrf_config']).relative_to('RRF/run/vsd'))


@dataclass(frozen=True)
class WorldConfiguration:
    machine: str
    machine_type: str
    layering: bool
    scene: str
    firmware_config: str
    backend: str
    base_radii_mm: tuple[float, ...]
    buildup_factor: float
    requested_json: str

    def effective(self):
        return {k: v for k, v in asdict(self).items() if k != 'requested_json'}


def world_configuration(root, args, *, backend):
    root = Path(root).resolve()
    allowed = {'machine', 'base_radii_mm', 'buildup_factor'}
    if not isinstance(args, dict) or set(args) - allowed:
        raise ValueError(f'World settings must be among {sorted(allowed)}')
    if 'machine' not in args:
        raise ValueError('A world needs a machine from machines/catalog.json')
    if backend not in BACKENDS:
        raise ValueError('Unknown physics backend')
    machine = machines.machine(args['machine'])
    layering = machine['line_layering']
    radii = args.get('base_radii_mm', [30.0])
    buildup = args.get('buildup_factor', LAYERED_BUILDUP if layering else 0.)
    validate_spools(layering, radii, buildup)
    radii = tuple(float(r) for r in (radii * 4 if len(radii) == 1 else radii))
    # The layer/ramp solver is a discrete winding model, not an arbitrary Q model.
    if layering and (radii != (30.,) * 4 or not math.isclose(buildup, LAYERED_BUILDUP, abs_tol=1e-12)):
        raise ValueError('Layered worlds support stock 30 mm base radii and buildup_factor=2/pi only')
    if not (root / machine['scene']).is_file() or not (root / machine['rrf_config']).is_file():
        raise ValueError('Scene and matching RRF configuration must exist')
    return WorldConfiguration(machine['name'], machines.scene(machine).get('machine_type'), layering,
                              machine['scene'], firmware_path(machine),
                              backend, radii, float(buildup), encode(args))


def prepare_world(root, directory, config):
    """Freeze sources and bake with the production JS baker before constructing physics."""
    from pxr import Sdf, Usd
    root, directory = Path(root), Path(directory)
    authored = repo_path(root, config.scene).read_text()
    (directory / 'authored-scene.usda').write_text(authored)
    design = authored
    layer = Sdf.Layer.CreateAnonymous('.usda')
    layer.ImportFromString(authored)
    stage = Usd.Stage.Open(layer)
    source_radii = []
    for axis in 'ABCD':
        prim = stage.GetPrimAtPath(f'/World/HangprinterScene/Spool{axis}')
        if not prim or not prim.GetAttribute('radius'):
            raise ValueError(f'Missing spool {axis} in scene')
        radius = prim.GetAttribute('radius').Get()
        number(f'source spool {axis} radius', radius, .001, .2)
        source_radii.append(radius * 1000)
    if tuple(source_radii) != config.base_radii_mm:
        if config.layering:
            raise ValueError('Layered source spool radii must match the stock 30 mm design')
        for axis, radius in zip('ABCD', config.base_radii_mm):
            prim = stage.GetPrimAtPath(f'/World/HangprinterScene/Spool{axis}')
            if not prim or not prim.GetAttribute('radius'):
                raise ValueError(f'Missing spool {axis} in scene')
            prim.GetAttribute('radius').Set(radius / 1000)
        design = stage.GetRootLayer().ExportToString()
    design_path = directory / 'design-scene.usda'
    design_path.write_text(design)
    command = ['node', str(root / 'scripts/bake_cable_scene.mjs')]
    if not config.layering:
        command.append('--no-line-layering')
    result = subprocess.run([*command, str(design_path), str(directory / 'scene.usda')],
                            cwd=root, capture_output=True, text=True, timeout=30)
    if result.returncode:
        raise ValueError(f'Scene bake failed: {result.stderr.strip()}')
    firmware = repo_path(root / 'RRF/run/vsd', config.firmware_config).read_text()
    if not re.search(r'^M669 K6\b', firmware, re.MULTILINE) or not re.search(r'^M669 N4\b', firmware, re.MULTILINE):
        raise ValueError('Matching RRF configuration must use four-anchor Hangprinter kinematics')
    q = re.search(r'^M666 Q([^\s]+) R([^\s;]+)', firmware, re.MULTILINE)
    firmware_source_hash = hashlib.sha256(firmware.encode()).hexdigest()
    if not q or not math.isclose(float(q[1]), config.buildup_factor, abs_tol=1e-12) or (
            not config.layering and list(map(float, q[2].split(':'))) != [30.] * 4):
        raise ValueError('Matching RRF configuration has incompatible Q/R settings')
    firmware_radii = list(map(float, q[2].split(':')))
    if not config.layering:
        firmware = firmware[:q.start()] + f'M666 Q0 R{":".join(map(str, config.base_radii_mm))}' + firmware[q.end():]
    (directory / 'firmware-config.g').write_text(firmware)
    return {'requested_settings': json.loads(config.requested_json),
            'effective_settings': config.effective(),
            'authored_scene_sha256': hashlib.sha256(authored.encode()).hexdigest(),
            'design_scene_sha256': hashlib.sha256(design.encode()).hexdigest(),
            'baked_scene_sha256': hashlib.sha256((directory / 'scene.usda').read_bytes()).hexdigest(),
            'firmware_configuration': {'source': config.firmware_config, 'buildup_factor': config.buildup_factor,
                'radii_mm': firmware_radii if config.layering else list(config.base_radii_mm)},
            'firmware_source_sha256': firmware_source_hash,
            'firmware_config_sha256': hashlib.sha256(firmware.encode()).hexdigest()}
