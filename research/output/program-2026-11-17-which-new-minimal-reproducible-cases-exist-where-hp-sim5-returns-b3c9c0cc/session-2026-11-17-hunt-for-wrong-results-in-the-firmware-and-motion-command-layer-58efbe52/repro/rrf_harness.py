"""Minimal harness: run the real RRF host firmware (rrf_simulator) in batch mode
on a small G-code file and parse the CAN motion capture it emits.

The simulator binary is the same artifact hp-sim5 uses (RRF/build/rrf_simulator).
No firmware source is modified; configs and G-code live in a scratch VSD tree.

CAN capture row format (RRF/host/can/CanCapture.cpp):
    seq, canAddress, whenToExecute, accelClocks, steadyClocks, decelClocks,
    <one steps column per drive in the message>, acceleration, deceleration

Step clock = 750 kHz (STEP_CLOCK_HZ_RRF_HOST = 48 MHz / 64).
"""
import json
import os
import pathlib
import re
import shutil
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[5]
SIM = ROOT / "RRF" / "build" / "rrf_simulator"
STEP_CLOCK_HZ = 750_000

CARTESIAN_CONFIG = """\
; Minimal cartesian config for RRF host simulator probes.
G21
G90
M83
M584 X40.0 Y41.0 Z42.0 E43.0 P3
M669 K0
M92 X80 Y80 Z400 E95.922
M201 X3000 Y3000 Z3000 E3000
M203 X18000 Y18000 Z18000 E18000
M204 P3000 T3000
M566 X300 Y300 Z300 E1200
M569 P40.0 S1
M569 P41.0 S1
M569 P42.0 S1
M569 P43.0 S1
M564 H0 S0
"""


def build_vsd(base: pathlib.Path) -> pathlib.Path:
    vsd = base / "vsd"
    (vsd / "sys").mkdir(parents=True, exist_ok=True)
    (vsd / "gcodes").mkdir(parents=True, exist_ok=True)
    (vsd / "logs").mkdir(parents=True, exist_ok=True)
    return vsd


def run_rrf(gcode_text, base: pathlib.Path, config_text=CARTESIAN_CONFIG,
            config_name="config_probe.g", extra_args=(), keep=True):
    """Run one G-code file through the real firmware. Returns a result dict."""
    base = pathlib.Path(base)
    if base.exists() and not keep:
        shutil.rmtree(base)
    vsd = build_vsd(base)
    (vsd / "sys" / config_name).write_text(config_text)
    gcode_path = vsd / "gcodes" / "probe.gcode"
    gcode_path.write_text(gcode_text)
    csv_path = base / "capture.csv"
    cmd = [str(SIM), "--vsd", str(vsd), "--gcode", str(gcode_path),
           "--can-log", str(csv_path), "-c", "sys/" + config_name]
    cmd.extend(extra_args)
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=55)
    out = proc.stdout + proc.stderr
    rows = parse_capture(csv_path) if csv_path.exists() else []
    return {
        "cmd": cmd,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
        "rows": rows,
        "final_position": parse_final_position(out),
        "errors": [ln for ln in out.splitlines()
                   if re.search(r"^Error|error:|is outside|insufficient|Unknown", ln)],
    }


def parse_capture(csv_path):
    rows = []
    with open(csv_path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("{"):
                continue
            parts = [p for p in line.split(",")]
            if len(parts) < 9:
                continue
            values = [float(p) for p in parts]
            num_drives = len(values) - 8
            rows.append({
                "seq": int(values[0]),
                "drive": int(values[1]),
                "when": values[2],
                "accel_clocks": values[3],
                "steady_clocks": values[4],
                "decel_clocks": values[5],
                "steps": values[6:6 + num_drives],
                "acceleration": values[6 + num_drives],
                "deceleration": values[7 + num_drives],
                "num_drives": num_drives,
                "raw": line,
            })
    return rows


def parse_final_position(out):
    match = re.search(r"Final machine position:(.*)", out)
    if not match:
        return None
    pos = {}
    for token in match.group(1).split():
        if "=" in token:
            key, value = token.split("=", 1)
            pos[key] = float(value)
    return pos


def step_totals(rows, drive=None):
    """Sum captured steps per CAN address (drive)."""
    totals = {}
    for row in rows:
        if drive is not None and row["drive"] != drive:
            continue
        totals[row["drive"]] = totals.get(row["drive"], 0.0) + sum(row["steps"])
    return totals


def total_time_s(row):
    return (row["accel_clocks"] + row["steady_clocks"] + row["decel_clocks"]) / STEP_CLOCK_HZ


if __name__ == "__main__":
    import sys
    base = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path("/tmp/opencode/rrf_probe_run")
    res = run_rrf("G92 X0 Y0 Z0\nG1 X10 F600\n", base)
    print(json.dumps({k: res[k] for k in ("returncode", "final_position", "errors")}, indent=2))
    print("steps:", step_totals(res["rows"]))
    for row in res["rows"]:
        print(row["raw"])
