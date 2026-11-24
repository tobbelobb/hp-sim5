#!/usr/bin/env python3
"""Generate a symmetric, physically radius-constrained hp4 scene and RRF config."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from pxr import Gf, Usd


ROOT = Path(__file__).resolve().parents[3]
SCENE_TEMPLATE = ROOT / "public/usd_scenes/hp4.usda"
CONFIG_TEMPLATE = ROOT / "RRF/run/vsd/sys/config_hp4_w_line_layers.g"
SCENE_ROOT = "/World/HangprinterScene"
BASE_ATTACHMENT_INRADIUS = 0.2


def _position(prim: Usd.Prim) -> Gf.Vec3d:
    attr = prim.GetAttribute("xformOp:translate")
    value = attr.Get() if attr else None
    if value is None:
        raise ValueError(f"missing xformOp:translate on {prim.GetPath()}")
    return Gf.Vec3d(value)


def _set_position(prim: Usd.Prim, position: Gf.Vec3d) -> None:
    prim.GetAttribute("xformOp:translate").Set(position)


def _replace_prefixed_line(text: str, prefix: str, replacement: str) -> str:
    lines = text.splitlines()
    matches = [index for index, line in enumerate(lines) if line.startswith(prefix)]
    if len(matches) != 1:
        raise ValueError(f"expected one {prefix!r} line, found {len(matches)}")
    lines[matches[0]] = replacement
    return "\n".join(lines) + "\n"


def _axis_unit(name: str) -> tuple[float, float]:
    if name.startswith("A"):
        return (0.0, -1.0)
    if name.startswith("B"):
        return (math.sqrt(3.0) / 2.0, 0.5)
    if name.startswith("C"):
        return (-math.sqrt(3.0) / 2.0, 0.5)
    raise ValueError(f"unsupported low anchor name {name}")


def _positive_radius_root(
    moving_position: Gf.Vec3d,
    fixed_z: float,
    unit: tuple[float, float],
    limit: float,
) -> float:
    dot = moving_position[0] * unit[0] + moving_position[1] * unit[1]
    constant = moving_position[0] ** 2 + moving_position[1] ** 2 + fixed_z**2 - limit**2
    discriminant = dot**2 - constant
    if discriminant <= 0:
        raise ValueError("no positive anchor-radius solution")
    return -dot + math.sqrt(discriminant)


def generate(
    attachment_inradius: float,
    boundary_radius: float,
    scene_output: Path,
    config_output: Path,
) -> None:
    if not (attachment_inradius > 0):
        raise ValueError("attachment inradius must be positive")
    if not (boundary_radius > 0):
        raise ValueError("boundary radius must be positive")

    stage = Usd.Stage.Open(str(SCENE_TEMPLATE))
    if stage is None:
        raise ValueError(f"could not open {SCENE_TEMPLATE}")
    root = stage.GetPrimAtPath(SCENE_ROOT)
    scale = attachment_inradius / BASE_ATTACHMENT_INRADIUS

    # Scale every mover-side low cable guide about the origin.
    for prim in root.GetChildren():
        name = prim.GetName()
        if name.startswith(("WheelD", "AttachD")):
            continue
        if not name.startswith(("Wheel", "Pinhole", "Corner")):
            continue
        pos = _position(prim)
        _set_position(prim, Gf.Vec3d(pos[0] * scale, pos[1] * scale, pos[2]))

    # Solve one common virtual low-anchor radius that places the worst fixed
    # top/bottom cable point on the requested sphere.
    anchor_specs: list[tuple[Usd.Prim, Usd.Prim, tuple[float, float], float]] = []
    low_radius_limits: list[float] = []
    for prim in root.GetChildren():
        name = prim.GetName()
        if not name.startswith("Anchor"):
            continue
        suffix = name.removeprefix("Anchor")
        pinhole = stage.GetPrimAtPath(f"{SCENE_ROOT}/Pinhole{suffix}")
        if not pinhole:
            raise ValueError(f"missing Pinhole{suffix}")
        unit = _axis_unit(suffix)
        fixed_z = _position(prim)[2]
        moving_pos = _position(pinhole)
        low_radius_limits.append(_positive_radius_root(moving_pos, fixed_z, unit, boundary_radius))
        anchor_specs.append((prim, pinhole, unit, fixed_z))
    virtual_low_radius = min(low_radius_limits)
    for anchor, pinhole, unit, fixed_z in anchor_specs:
        moving_pos = _position(pinhole)
        _set_position(
            anchor,
            Gf.Vec3d(
                moving_pos[0] + virtual_low_radius * unit[0],
                moving_pos[1] + virtual_low_radius * unit[1],
                fixed_z,
            ),
        )

    # Keep each low spool 0.4 m radially outside its fixed anchor-pair center.
    for axis in "ABC":
        spool = stage.GetPrimAtPath(f"{SCENE_ROOT}/Spool{axis}")
        unit = _axis_unit(axis)
        radial_center = virtual_low_radius + 1.2 * attachment_inradius + 0.4
        pos = _position(spool)
        _set_position(spool, Gf.Vec3d(unit[0] * radial_center, unit[1] * radial_center, pos[2]))

    # Scale the D-line XY footprint. Its fixed top guide plane is raised as far
    # as the same radius constraint allows; mover-side bottom guides stay at z=0.1.
    d_top_prims: list[Usd.Prim] = []
    d_attach_prims: list[Usd.Prim] = []
    max_d_xy = 0.0
    for prim in root.GetChildren():
        name = prim.GetName()
        if not (name.startswith("WheelD") or name.startswith("AttachD")):
            continue
        pos = _position(prim)
        scaled = Gf.Vec3d(pos[0] * scale, pos[1] * scale, pos[2])
        if name.startswith("AttachD") or name.endswith("_top"):
            d_top_prims.append(prim)
            max_d_xy = max(max_d_xy, math.hypot(scaled[0], scaled[1]))
            if name.startswith("AttachD"):
                d_attach_prims.append(prim)
        _set_position(prim, scaled)
    top_z = math.sqrt(boundary_radius**2 - max_d_xy**2)
    for prim in d_top_prims:
        pos = _position(prim)
        _set_position(prim, Gf.Vec3d(pos[0], pos[1], top_z))
    spool_d = stage.GetPrimAtPath(f"{SCENE_ROOT}/SpoolD")
    _set_position(spool_d, Gf.Vec3d(0.0, 0.0, top_z + 0.0065))
    virtual_top_radius = top_z - 0.1

    fixed_anchor_radii = [
        math.sqrt(sum(component * component for component in _position(anchor)))
        for anchor, *_ in anchor_specs
    ]
    fixed_d_radii = [
        math.sqrt(sum(component * component for component in _position(prim)))
        for prim in d_top_prims
    ]
    max_fixed_radius = max(fixed_anchor_radii + fixed_d_radii)
    if max_fixed_radius > boundary_radius + 1e-9:
        raise ValueError(f"fixed cable anchor radius {max_fixed_radius:.9f} exceeds boundary")

    scene_output.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(scene_output))

    low_guy_lengths: list[float] = []
    for axis in "ABC":
        spool_pos = _position(stage.GetPrimAtPath(f"{SCENE_ROOT}/Spool{axis}"))
        anchor_positions = [
            _position(stage.GetPrimAtPath(f"{SCENE_ROOT}/Anchor{axis}{side}_top"))
            for side in ("L", "R")
        ]
        low_guy_lengths.append(
            sum(math.dist(tuple(spool_pos), tuple(anchor)) for anchor in anchor_positions) / 2.0
        )
    spool_d_pos = _position(spool_d)
    d_guy_length = sum(
        math.dist(tuple(spool_d_pos), tuple(_position(prim))) for prim in d_attach_prims
    ) / len(d_attach_prims)

    low_mm = virtual_low_radius * 1000.0
    bx_mm = math.sqrt(3.0) * 0.5 * low_mm
    by_mm = 0.5 * low_mm
    config = CONFIG_TEMPLATE.read_text()
    config = _replace_prefixed_line(
        config,
        "M669 A",
        (
            f"M669 A0.0:{-low_mm:.6f}:-280.0 "
            f"B{bx_mm:.6f}:{by_mm:.6f}:-280.0 "
            f"C{-bx_mm:.6f}:{by_mm:.6f}:-280.0 "
            f"D0.0:0.0:{virtual_top_radius * 1000.0:.6f}"
        ),
    )
    guy_mm = [value * 1000.0 for value in low_guy_lengths] + [d_guy_length * 1000.0]
    config = _replace_prefixed_line(
        config,
        "M666 Y",
        "M666 Y" + ":".join(f"{value:.6f}" for value in guy_mm),
    )
    config_output.parent.mkdir(parents=True, exist_ok=True)
    config_output.write_text(config)

    print(
        f"scene={scene_output} config={config_output} max_fixed_radius={max_fixed_radius:.6f}m "
        f"virtual_low_radius={virtual_low_radius:.6f}m virtual_top_radius={virtual_top_radius:.6f}m "
        f"attachment_inradius={attachment_inradius:.6f}m"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--attachment-inradius", type=float, required=True)
    parser.add_argument("--boundary-radius", type=float, default=2.0)
    parser.add_argument("--scene-output", type=Path, required=True)
    parser.add_argument("--config-output", type=Path, required=True)
    args = parser.parse_args()
    generate(
        args.attachment_inradius,
        args.boundary_radius,
        args.scene_output,
        args.config_output,
    )


if __name__ == "__main__":
    main()
