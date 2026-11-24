#!/usr/bin/env python3
"""Generate a symmetric six-anchor Slideprinter scene and matching RRF config."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from pxr import Gf, Usd


ROOT = Path(__file__).resolve().parents[3]
SCENE_TEMPLATE = ROOT / "public/usd_scenes/slideprinter_multi_unit.usda"
CONFIG_TEMPLATE = ROOT / "RRF/run/vsd/sys/config_slideprinter_w_line_layers.g"
SCENE_ROOT = "/World/SlideprinterScene"


def _set_position(stage: Usd.Stage, name: str, x: float, y: float) -> None:
    prim = stage.GetPrimAtPath(f"{SCENE_ROOT}/{name}")
    attr = prim.GetAttribute("xformOp:translate")
    if not attr:
        raise ValueError(f"missing xformOp:translate on {prim.GetPath()}")
    attr.Set(Gf.Vec3d(x, y, 0.0))


def _positions(virtual_radius: float, attachment_inradius: float) -> dict[str, tuple[float, float]]:
    sqrt3 = math.sqrt(3.0)
    d = sqrt3 * attachment_inradius
    ux = sqrt3 / 2.0

    attach_al = (-d, -attachment_inradius)
    attach_ar = (d, -attachment_inradius)
    attach_br = (0.0, 2.0 * attachment_inradius)

    return {
        "AttachAL": attach_al,
        "AttachAR": attach_ar,
        "AttachBR": attach_br,
        "AnchorAL": (attach_al[0], attach_al[1] - virtual_radius),
        "AnchorAR": (attach_ar[0], attach_ar[1] - virtual_radius),
        "AnchorBL": (attach_ar[0] + ux * virtual_radius, attach_ar[1] + 0.5 * virtual_radius),
        "AnchorBR": (attach_br[0] + ux * virtual_radius, attach_br[1] + 0.5 * virtual_radius),
        "AnchorCL": (attach_br[0] - ux * virtual_radius, attach_br[1] + 0.5 * virtual_radius),
        "AnchorCR": (attach_al[0] - ux * virtual_radius, attach_al[1] + 0.5 * virtual_radius),
    }


def _replace_prefixed_line(text: str, prefix: str, replacement: str) -> str:
    lines = text.splitlines()
    matches = [index for index, line in enumerate(lines) if line.startswith(prefix)]
    if len(matches) != 1:
        raise ValueError(f"expected one {prefix!r} line, found {len(matches)}")
    lines[matches[0]] = replacement
    return "\n".join(lines) + "\n"


def generate(
    virtual_radius: float,
    attachment_inradius: float,
    scene_output: Path,
    config_output: Path,
) -> None:
    positions = _positions(virtual_radius, attachment_inradius)
    fixed_anchor_names = [name for name in positions if name.startswith("Anchor")]
    max_anchor_radius = max(math.hypot(*positions[name]) for name in fixed_anchor_names)
    if max_anchor_radius > 2.0 + 1e-9:
        raise ValueError(f"fixed anchor radius {max_anchor_radius:.6f} m exceeds 2.0 m")

    stage = Usd.Stage.Open(str(SCENE_TEMPLATE))
    if stage is None:
        raise ValueError(f"could not open {SCENE_TEMPLATE}")
    for name, (x, y) in positions.items():
        _set_position(stage, name, x, y)
    scene_output.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(scene_output))

    radius_mm = virtual_radius * 1000.0
    anchor_bx_mm = math.sqrt(3.0) * 0.5 * radius_mm
    anchor_by_mm = 0.5 * radius_mm
    fixed_center_radius = virtual_radius + attachment_inradius
    tangential_offset = math.sqrt(3.0) * attachment_inradius
    guy_length_mm = math.hypot(2.5 - fixed_center_radius, tangential_offset) * 1000.0

    config = CONFIG_TEMPLATE.read_text()
    config = _replace_prefixed_line(
        config,
        "M669 A",
        (
            f"M669 A0.0:{-radius_mm:.6f}:0.0 "
            f"B{anchor_bx_mm:.6f}:{anchor_by_mm:.6f}:0.0 "
            f"C{-anchor_bx_mm:.6f}:{anchor_by_mm:.6f}:0.0"
        ),
    )
    config = _replace_prefixed_line(
        config,
        "M666 Y",
        f"M666 Y{guy_length_mm:.6f}:{guy_length_mm:.6f}:{guy_length_mm:.6f}",
    )
    config_output.parent.mkdir(parents=True, exist_ok=True)
    config_output.write_text(config)

    print(
        f"scene={scene_output} config={config_output} "
        f"max_anchor_radius={max_anchor_radius:.6f}m virtual_radius={virtual_radius:.6f}m "
        f"attachment_inradius={attachment_inradius:.6f}m guy_length={guy_length_mm:.3f}mm"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--virtual-radius", type=float, required=True)
    parser.add_argument("--attachment-inradius", type=float, required=True)
    parser.add_argument("--scene-output", type=Path, required=True)
    parser.add_argument("--config-output", type=Path, required=True)
    args = parser.parse_args()
    generate(
        args.virtual_radius,
        args.attachment_inradius,
        args.scene_output,
        args.config_output,
    )


if __name__ == "__main__":
    main()
