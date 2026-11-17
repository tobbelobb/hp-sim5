"""Build the experiment-only collector that measures a second free line.

Production files are never edited.  Two source files are copied with their
relative imports rewritten to absolute file:// URLs, then the behaviour copy is
patched explicitly.  The CLI is used only by the treatment arm; the baseline and
the rough-bootstrap phase use the untouched production collector.

The patch is deliberately narrow:
  * preload (2 * force_mid) and measurement (sensor collection force) include
    the one remaining free low axis;
  * main pullout and idle keep force_low;
  * every point records extra_sensor_anchor / extra_sensor_force_n /
    extra_sensor_force_mode, and every sweep records its policy.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SESSION = HERE.parent
REPO = next(p for p in SESSION.parents if (p / "autocal/control").exists())
OUT = HERE / "extra_range_collector"

SOURCES = [
    REPO / "autocal/control/cli/collect_sweep_data.mjs",
    REPO / "autocal/control/behaviors/sweep_data_collection.mjs",
]

SPECIFIER = re.compile(r"(from\s+|import\s*\(\s*)'(\.[^']*)'")

HEADER = """// EXPERIMENT COPY: built by evidence/build_extra_range_collector.py.
// Measures the one remaining free low axis at the sensor force during preload
// and measurement. Main pullout and idle remain at force_low. Production files
// are untouched.
"""


def absolutize(text: str, source: Path) -> str:
    def repl(match: re.Match) -> str:
        prefix, rel = match.group(1), match.group(2)
        target = (source.parent / rel).resolve()
        return f"{prefix}'file://{target}'"
    return SPECIFIER.sub(repl, text)


def replace_once(text: str, old: str, new: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected exactly one occurrence, found {count}:\n{old}")
    return text.replace(old, new)


def patch_behavior(text: str) -> str:
    # Per-sweep fixed-target groups: needed only by the short matched preflight
    # (one collector invocation, several test poses). A single group behaves
    # exactly like production, so the full autocal loop is unaffected.
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
        """  // Experiment: one ';'-separated group per sweep. A single group is the
  // production case.
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
    text = replace_once(
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
    text = replace_once(
        text,
        "function buildFixedTargetByAnchor(fixedAnchors, fixedTargets, motorCount) {",
        """const EXTRA_SENSOR_POLICY = 'measurement_only_sensor_force';

function extraRangeAnchor(motorIds, fixedAnchors, driveAnchor, sensorAnchor, forbiddenForceAnchors) {
  const used = new Set([...(fixedAnchors ?? []), ...(forbiddenForceAnchors ?? []), driveAnchor, sensorAnchor]
    .filter((idx) => Number.isFinite(idx)));
  return motorIds.map((_, idx) => idx).find((idx) => !used.has(idx)) ?? null;
}

function buildFixedTargetByAnchor(fixedAnchors, fixedTargets, motorCount) {""",
    )
    text = replace_once(
        text,
        """  const { driveAnchor, sensorAnchor } = sweepConfig;
  const driveAxis = axes[driveAnchor];
  const forbidden = new Set(forbiddenForceAnchors ?? []);""",
        """  const { driveAnchor, sensorAnchor } = sweepConfig;
  // Experiment: the one remaining free axis is measured at the same sensor
  // force during preload/measurement, but stays at forceLow during pullout/idle.
  const extraAnchor = extraRangeAnchor(motorIds, fixedAnchors, driveAnchor, sensorAnchor, forbiddenForceAnchors);
  const extraSensors = Number.isInteger(extraAnchor) ? [sensorAnchor, extraAnchor] : [sensorAnchor];
  console.log(`; extra-range experiment: measuring anchors [${extraSensors.join(', ')}] at the sensor force`);
  const driveAxis = axes[driveAnchor];
  const forbidden = new Set(forbiddenForceAnchors ?? []);""",
    )
    text = replace_once(
        text,
        "    return idx === sensorAnchor ? forceMid * 2.0 : forceLow;",
        "    return extraSensors.includes(idx) ? forceMid * 2.0 : forceLow;",
    )
    text = replace_once(
        text,
        "        motorIds, driveAnchor, sensorAnchors: [sensorAnchor], fixedAnchors, forbiddenForceAnchors,",
        "        motorIds, driveAnchor, sensorAnchors: extraSensors, fixedAnchors, forbiddenForceAnchors,",
    )
    text = replace_once(
        text,
        "        return idx === sensorAnchor ? sensorForce : forceLow;",
        "        return extraSensors.includes(idx) ? sensorForce : forceLow;",
    )
    text = text.replace("sensorAnchors: [sensorAnchor],", "sensorAnchors: extraSensors,")
    if text.count("sensorAnchors: extraSensors") != 3:
        raise RuntimeError("expected three sensorAnchors replacements")
    text = replace_once(
        text,
        """      sensor_force_n: resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce }),
      spool_reserve_method: reserveProbe.accepted ? 'local_encoder_coupling' : 'half_turn',
    };""",
        """      sensor_force_n: resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce }),
      spool_reserve_method: reserveProbe.accepted ? 'local_encoder_coupling' : 'half_turn',
      ...(Number.isInteger(extraAnchor) ? {
        extra_sensor_anchor: extraAnchor,
        extra_sensor_force_n: resolveCollectionForce({ forceMid, forceMax, sensorCollectionForce }),
        extra_sensor_force_mode: EXTRA_SENSOR_POLICY,
      } : {}),
    };""",
    )
    text = replace_once(
        text,
        """    const driveRange = driveRangeValues.length > 0
      ? {
        start: Math.min(...driveRangeValues),
        end: Math.max(...driveRangeValues),
        unit: 'mm',
      }
      : null;""",
        """    const driveRange = driveRangeValues.length > 0
      ? {
        start: Math.min(...driveRangeValues),
        end: Math.max(...driveRangeValues),
        unit: 'mm',
      }
      : null;
    const sweepExtraAnchor = extraRangeAnchor(
      motorIds, sweepConfig.fixedAnchors, sweepConfig.driveAnchor, sweepConfig.sensorAnchor,
      getForceForbiddenAnchors(machineConfig),
    );""",
    )
    text = replace_once(
        text,
        """        spool_reserves: spoolReserves,
        settle_ms: settleMs,
      },""",
        """        spool_reserves: spoolReserves,
        settle_ms: settleMs,
        extra_sensor_anchors: Number.isInteger(sweepExtraAnchor) ? [sweepExtraAnchor] : [],
        extra_sensor_policy: EXTRA_SENSOR_POLICY,
      },""",
    )
    text = replace_once(
        text,
        "      noise_model: { project_zero_tension: false },",
        """      noise_model: { project_zero_tension: false },
      extra_sensor_policy: {
        mode: EXTRA_SENSOR_POLICY,
        version: 1,
        preload_force: 'force_mid * 2',
        measurement_force: 'sensor_collection_force',
        pullout_force: 'force_low',
        idle_force: 'force_low',
      },""",
    )
    return text


def main() -> None:
    OUT.mkdir(exist_ok=True)
    for source in SOURCES:
        text = absolutize(source.read_text(), source)
        if source.name == "sweep_data_collection.mjs":
            text = patch_behavior(text)
        else:
            production = (REPO / "autocal/control/behaviors/sweep_data_collection.mjs").resolve()
            local = (OUT / "sweep_data_collection.mjs").resolve()
            if text.count(f"file://{production}") < 1:
                raise RuntimeError("CLI does not import the production behaviour module")
            text = text.replace(f"file://{production}", f"file://{local}")
            text = replace_once(
                text,
                "const SOURCE_FILE_LABEL = 'autocal/control/cli/collect_sweep_data.mjs';",
                "const SOURCE_FILE_LABEL = 'experiment/extra_range_collector/collect_sweep_data.mjs';",
            )
        if text.startswith("#!"):
            first, rest = text.split("\n", 1)
            text = first + "\n" + HEADER + rest
        else:
            text = HEADER + text
        target = OUT / source.name
        target.write_text(text)
        print(f"wrote {target.relative_to(SESSION)}")


if __name__ == "__main__":
    sys.exit(main())
