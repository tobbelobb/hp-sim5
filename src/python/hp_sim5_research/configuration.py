"""Validated, immutable settings for one coordinated world generation."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

from .experiments import encode, repo_path

BACKENDS = ('native-python', 'headless-js', 'native-warp', 'native-warp-cuda')
LAYERED_BUILDUP = 2 / math.pi


def number(name, value, low, high, *, integer=False):
    if (type(value) not in ((int,) if integer else (int, float)) or
            not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f'{name} must be {"an integer" if integer else "a number"} in [{low}, {high}]')
    return value


def validate_spools(machine, layering, radii, buildup):
    if machine not in ('hp3', 'hp4'):
        raise ValueError('machine must be hp3 or hp4')
    if type(layering) is not bool:
        raise ValueError('layering must be boolean')
    if not isinstance(radii, list) or len(radii) not in (1, 4):
        raise ValueError('base_radii_mm must contain one or four radii')
    for radius in radii:
        number('base_radii_mm', radius, 1, 200)
    number('buildup_factor', buildup, 0, 2)
    if not layering and buildup != 0:
        raise ValueError('Nonzero or fitted buildup requires layering')


def firmware_path(machine, layering):
    return f'sys/config_{machine}{"_w_line_layers" if layering else ""}.g'


@dataclass(frozen=True)
class WorldConfiguration:
    machine: str
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
    allowed = {'machine', 'layering', 'scene', 'firmware_config', 'base_radii_mm', 'buildup_factor'}
    if not isinstance(args, dict) or set(args) - allowed:
        raise ValueError(f'World settings must be among {sorted(allowed)}')
    if backend not in BACKENDS:
        raise ValueError('Unknown physics backend')
    machine, layering = args.get('machine', 'hp4'), args.get('layering', True)
    radii = args.get('base_radii_mm', [30.0])
    buildup = args.get('buildup_factor', LAYERED_BUILDUP if layering is True else 0.)
    validate_spools(machine, layering, radii, buildup)
    radii = tuple(float(r) for r in (radii * 4 if len(radii) == 1 else radii))
    # The layer/ramp solver is a discrete winding model, not an arbitrary Q model.
    if layering and (radii != (30.,) * 4 or not math.isclose(buildup, LAYERED_BUILDUP, abs_tol=1e-12)):
        raise ValueError('Layered worlds support stock 30 mm base radii and buildup_factor=2/pi only')
    scene = args.get('scene', f'public/usd_scenes/{machine}_rigid_body.usda')
    firmware = args.get('firmware_config', firmware_path(machine, layering))
    for name, value in [('scene', scene), ('firmware_config', firmware)]:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{name} must be a repository path')
    scene_path = repo_path(Path(root), scene)
    config_path = repo_path(Path(root) / 'RRF/run/vsd', firmware)
    if scene_path != repo_path(Path(root), f'public/usd_scenes/{machine}_rigid_body.usda'):
        raise ValueError('Only the matching stock HP3/HP4 scene is supported; custom anchor designs are unsupported')
    if config_path != repo_path(Path(root) / 'RRF/run/vsd', firmware_path(machine, layering)):
        raise ValueError('firmware_config must match machine and layering')
    if not scene_path.is_file() or not config_path.is_file():
        raise ValueError('Scene and matching RRF configuration must exist')
    return WorldConfiguration(machine, layering, str(scene_path.relative_to(root)),
                              str(config_path.relative_to(Path(root) / 'RRF/run/vsd')),
                              backend, radii, float(buildup), encode(args))


def prepare_world(root, directory, config):
    """Freeze sources and bake with the production JS baker before constructing physics."""
    from usd.cable_scene_loader import open_stage
    root, directory = Path(root), Path(directory)
    authored = repo_path(root, config.scene).read_text()
    (directory / 'authored-scene.usda').write_text(authored)
    design = authored
    stage = open_stage(authored)
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
    expected_radii = ([39.1845 if config.machine == 'hp3' else 46.6045] * 4
                      if config.layering else [30.] * 4)
    firmware_source_hash = hashlib.sha256(firmware.encode()).hexdigest()
    if not q or not math.isclose(float(q[1]), config.buildup_factor, abs_tol=1e-12) or list(map(float, q[2].split(':'))) != expected_radii:
        raise ValueError('Matching RRF configuration has incompatible Q/R settings')
    if not config.layering:
        firmware = firmware[:q.start()] + f'M666 Q0 R{":".join(map(str, config.base_radii_mm))}' + firmware[q.end():]
    (directory / 'firmware-config.g').write_text(firmware)
    return {'requested_settings': json.loads(config.requested_json),
            'effective_settings': config.effective(),
            'authored_scene_sha256': hashlib.sha256(authored.encode()).hexdigest(),
            'design_scene_sha256': hashlib.sha256(design.encode()).hexdigest(),
            'baked_scene_sha256': hashlib.sha256((directory / 'scene.usda').read_bytes()).hexdigest(),
            'firmware_configuration': {'source': config.firmware_config, 'buildup_factor': config.buildup_factor,
                'radii_mm': expected_radii if config.layering else list(config.base_radii_mm)},
            'firmware_source_sha256': firmware_source_hash,
            'firmware_config_sha256': hashlib.sha256(firmware.encode()).hexdigest()}
