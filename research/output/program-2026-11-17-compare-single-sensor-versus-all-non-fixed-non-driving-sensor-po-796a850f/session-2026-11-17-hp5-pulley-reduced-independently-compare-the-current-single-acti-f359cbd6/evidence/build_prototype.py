"""Build experiment-local collector variants. Production files are never edited.

Two source files are copied per variant with their relative imports rewritten to
absolute file:// URLs, so the copies run in place:

  - autocal/control/cli/collect_sweep_data.mjs
  - autocal/control/behaviors/sweep_data_collection.mjs

Variants:
  prototype-control/  per-sweep fixed-target groups (needed because a single
                      global target keeps the top-anchor length constant, which
                      leaves the top anchor height unidentifiable and is not how
                      production's rough sweeps vary poses).
  prototype/          the above plus measuring BOTH non-fixed, non-driving lines
                      at the sensor measurement force (preload and measurement).

Invoke the copied CLI exactly like the production one. Example:
  node evidence/prototype-control/collect_sweep_data.mjs --machineType ... --sim ...
"""
from __future__ import annotations

import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
SESSION = HERE.parent.parent
REPO = next(p for p in SESSION.parents if (p / "autocal/control").exists())

SOURCES = [
    REPO / "autocal/control/cli/collect_sweep_data.mjs",
    REPO / "autocal/control/behaviors/sweep_data_collection.mjs",
]

SPECIFIER = re.compile(r"(from\s+|import\s*\(\s*)'(\.[^']*)'")


def absolutize(text: str, source: Path) -> str:
    def repl(match: re.Match) -> str:
        prefix, rel = match.group(1), match.group(2)
        target = (source.parent / rel).resolve()
        return f"{prefix}'file://{target}'"
    return SPECIFIER.sub(repl, text)


def replace_once(text: str, old: str, new: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"expected exactly one occurrence of:\n{old}\nfound {text.count(old)}")
    return text.replace(old, new)


def patch_target_groups(text: str) -> str:
    """Accept ';'-separated fixed-target groups, one per sweep."""
    text = replace_once(
        text,
        """  if (raw.includes(';')) {
    throw new Error('Multiple fixed-target groups are no longer supported (super sweep removed).');
  }
  const values = raw
    .split(',')
    .map((entry) => entry.trim())
    .filter(Boolean)
    .map((entry) => parseFloat(entry));
  if (values.length === 0) {
    return null;
  }
  if (values.some((val) => !Number.isFinite(val))) {
    throw new Error('Fixed targets must be numeric values.');
  }
  return values;""",
        """  // Experiment: one ';'-separated group per sweep, so later sweeps can vary
  // the top-anchor length (pose diversity in Z) within one collector run.
  const groups = raw.split(';')
    .map((group) => group.split(',').map((entry) => entry.trim()).filter(Boolean).map((entry) => parseFloat(entry)))
    .filter((group) => group.length > 0);
  if (groups.length === 0) {
    return null;
  }
  if (groups.some((group) => group.some((val) => !Number.isFinite(val)))) {
    throw new Error('Fixed targets must be numeric values.');
  }
  return groups;""",
    )
    return replace_once(
        text,
        """    const fixedTargets = resolveFixedTargets(
      sweepConfig.fixedAnchors,
      fixedTargetsSpec,
      maxTravelMm,
      machineConfig,
    );""",
        """    const fixedTargetGroup = Array.isArray(fixedTargetsSpec) && Array.isArray(fixedTargetsSpec[0])
      ? fixedTargetsSpec[Math.min(sweepOrdinal - 1, fixedTargetsSpec.length - 1)]
      : fixedTargetsSpec;
    const fixedTargets = resolveFixedTargets(
      sweepConfig.fixedAnchors,
      fixedTargetGroup,
      maxTravelMm,
      machineConfig,
    );""",
    )


def patch_two_sensor(text: str) -> str:
    text = replace_once(
        text,
        "  const { driveAnchor, sensorAnchor } = sweepConfig;\n  const driveAxis = axes[driveAnchor];",
        "  const { driveAnchor, sensorAnchor } = sweepConfig;\n"
        "  // Experiment: measure both non-fixed, non-driving lines.\n"
        "  const activeAnchors = new Set([...(fixedAnchors ?? []), ...(forbiddenForceAnchors ?? []),\n"
        "    driveAnchor, sensorAnchor]);\n"
        "  const redundantAnchor = motorIds.map((_, idx) => idx).find(idx => !activeAnchors.has(idx));\n"
        "  const extraSensors = Number.isInteger(redundantAnchor) ? [sensorAnchor, redundantAnchor] : [sensorAnchor];\n"
        "  const driveAxis = axes[driveAnchor];",
    )
    text = replace_once(text,
                        "    return idx === sensorAnchor ? forceMid * 2.0 : forceLow;",
                        "    return extraSensors.includes(idx) ? forceMid * 2.0 : forceLow;")
    text = replace_once(
        text,
        "        motorIds, driveAnchor, sensorAnchors: [sensorAnchor], fixedAnchors, forbiddenForceAnchors,",
        "        motorIds, driveAnchor, sensorAnchors: extraSensors, fixedAnchors, forbiddenForceAnchors,",
    )
    text = replace_once(text,
                        "        return idx === sensorAnchor ? sensorForce : forceLow;",
                        "        return extraSensors.includes(idx) ? sensorForce : forceLow;")
    text = replace_once(text,
                        "          sensorAnchors: [sensorAnchor],\n          fixedAnchors,",
                        "          sensorAnchors: extraSensors,\n          fixedAnchors,")
    text = replace_once(text,
                        "      sensorAnchors: [sensorAnchor],\n      fixedAnchors,",
                        "      sensorAnchors: extraSensors,\n      fixedAnchors,")
    return text


def write_variant(name: str, two_sensor: bool) -> None:
    out = HERE / name
    out.mkdir(exist_ok=True)
    for source in SOURCES:
        target = out / source.name
        text = absolutize(source.read_text(), source)
        if source.name == "sweep_data_collection.mjs":
            text = patch_target_groups(text)
            if two_sensor:
                text = patch_two_sensor(text)
                header = ("// EXPERIMENT COPY: see evidence/build_prototype.py. Measures the redundant\n"
                          "// non-fixed, non-driving line at the sensor force and allows per-sweep targets.\n")
            else:
                header = ("// EXPERIMENT COPY: see evidence/build_prototype.py. Control variant:\n"
                          "// per-sweep fixed-target groups only.\n")
        else:
            production = (REPO / "autocal/control/behaviors/sweep_data_collection.mjs").resolve()
            prototype = (out / "sweep_data_collection.mjs").resolve()
            text = text.replace(f"file://{production}", f"file://{prototype}")
            header = f"// EXPERIMENT COPY: see evidence/build_prototype.py. CLI for variant {name}.\n"
        if text.startswith("#!"):
            first, rest = text.split("\n", 1)
            text = first + "\n" + header + rest
        else:
            text = header + text
        target.write_text(text)
        print(f"wrote {target.relative_to(REPO)}")


def main() -> None:
    write_variant("prototype-control", two_sensor=False)
    write_variant("prototype", two_sensor=True)


if __name__ == "__main__":
    main()
