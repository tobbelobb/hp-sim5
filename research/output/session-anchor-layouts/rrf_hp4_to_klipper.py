#!/usr/bin/env python3
"""Translate hp4 anchor and guy-wire values from an RRF config into Klipper."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
KLIPPER_TEMPLATE = ROOT / "public/klipper/hp4/printer-hp4-linux-mcu-with-buildup.cfg"


def _parse_xyz(value: str) -> tuple[float, float, float]:
    parts = value.split(":")
    if len(parts) != 3:
        raise ValueError(f"expected X:Y:Z, got {value!r}")
    return tuple(float(part) for part in parts)  # type: ignore[return-value]


def _replace_single(pattern: str, replacement: str, text: str) -> str:
    result, count = re.subn(pattern, replacement, text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise ValueError(f"pattern matched {count} times: {pattern}")
    return result


def convert(rrf_config: Path, output: Path) -> None:
    rrf = rrf_config.read_text()
    match = re.search(
        r"^M669\s+A(?P<A>\S+)\s+B(?P<B>\S+)\s+C(?P<C>\S+)\s+D(?P<D>\S+)",
        rrf,
        flags=re.MULTILINE,
    )
    if not match:
        raise ValueError("could not find hp4 M669 anchor line")
    anchors = {axis.lower(): _parse_xyz(match.group(axis)) for axis in "ABCD"}
    guy_match = re.search(r"^M666\s+Y(?P<values>\S+)", rrf, flags=re.MULTILINE)
    if not guy_match:
        raise ValueError("could not find M666 Y line")
    guy_values = [float(value) for value in guy_match.group("values").split(":")]
    if len(guy_values) != 4:
        raise ValueError("expected four guy-wire lengths")

    config = KLIPPER_TEMPLATE.read_text()
    config = _replace_single(
        r"^winch_guy_wire_lengths:.*$",
        "winch_guy_wire_lengths: " + ", ".join(f"{value:.6f}" for value in guy_values),
        config,
    )
    for axis, coords in anchors.items():
        section_pattern = rf"(?ms)(^\[stepper_{axis}\]\n.*?)(?=^\[|\Z)"
        section_match = re.search(section_pattern, config)
        if not section_match:
            raise ValueError(f"missing [stepper_{axis}] section")
        section = section_match.group(1)
        for coord_name, value in zip(("x", "y", "z"), coords):
            section = _replace_single(
                rf"^anchor_{coord_name}:.*$",
                f"anchor_{coord_name}: {value:.6f}",
                section,
            )
        config = config[: section_match.start(1)] + section + config[section_match.end(1) :]

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(config)
    print(f"klipper_config={output}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("rrf_config", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    convert(args.rrf_config, args.output)


if __name__ == "__main__":
    main()
