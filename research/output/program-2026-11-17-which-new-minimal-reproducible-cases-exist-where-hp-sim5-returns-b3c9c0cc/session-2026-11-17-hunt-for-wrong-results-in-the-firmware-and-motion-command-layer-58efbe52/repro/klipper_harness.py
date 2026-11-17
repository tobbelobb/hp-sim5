"""Minimal harness: run the real Klipper host (klippy.py batch/debug mode) on a
small G-code file with the repo's winch configuration, then parse the resulting
MCU command stream (public/klipper/linux_mcu/klipper.dict).

klippy batch mode is the same G-code parser + move planner + step compressor the
hp-sim5 Klipper emulator drives; no firmware source is modified. The only edit is
to the pin chip number in a *copy* of the config, because the checked-in linux MCU
dictionary only knows gpiochip0..8 and the shipped config has a placeholder.
"""
import pathlib
import re
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[5]
KLIPPY = ROOT / "klipper" / "klippy" / "klippy.py"
PARSE = ROOT / "klipper" / "klippy" / "parsedump.py"
DICT = ROOT / "public" / "klipper" / "linux_mcu" / "klipper.dict"
SHIPPED_SLIDEPRINTER = ROOT / "public" / "klipper" / "slideprinter" / "printer-slideprinter-linux-mcu.cfg"
SHIPPED_HP4 = ROOT / "public" / "klipper" / "hp4" / "printer-hp4-linux-mcu.cfg"

CLOCK_HZ = 50_000_000


def make_config(base: pathlib.Path, shipped=SHIPPED_SLIDEPRINTER, chip=2,
                overrides=None, extra_sections=""):
    text = shipped.read_text()
    text = text.replace("REPLACE_ME_WITH_FAKE_GPIO_CHIP_NUMBER", str(chip))
    if overrides:
        for key, value in overrides.items():
            text = re.sub(rf"^{key}:\s*.*$", f"{key}: {value}", text, flags=re.M)
    if extra_sections:
        text += "\n" + extra_sections + "\n"
    path = base / "printer.cfg"
    base.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def run_klipper(gcode_text, base: pathlib.Path, cfg_path=None, **cfg_kwargs):
    base = pathlib.Path(base)
    base.mkdir(parents=True, exist_ok=True)
    cfg_path = cfg_path or make_config(base, **cfg_kwargs)
    gcode_path = base / "probe.gcode"
    gcode_path.write_text(gcode_text)
    serial = base / "probe.serial"
    cmd = [str(ROOT / ".venv" / "bin" / "python"), str(KLIPPY), str(cfg_path),
           "-i", str(gcode_path), "-o", str(serial), "-v", "-d", str(DICT)]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=55)
    if proc.returncode != 0 and "-i" in cmd:
        # klippy batch mode occasionally fails on a loaded machine; one retry
        # keeps the comparison scripts deterministic without masking a real error.
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=55)
    text = ""
    if serial.exists():
        parsed = subprocess.run(
            [str(ROOT / ".venv" / "bin" / "python"), str(PARSE), str(DICT), str(serial)],
            capture_output=True, text=True, timeout=55)
        text = parsed.stdout
    return {
        "cmd": cmd,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
        "mcu_commands": text,
        "steps": parse_steps(text),
        "errors": [ln for ln in (proc.stdout + proc.stderr).splitlines()
                   if re.search(r"Error|error|Unknown command|Invalid|must be", ln)],
    }


def parse_steps(text):
    """Signed step count per stepper oid from the queue_step stream."""
    direction = {}
    steps = {}
    for line in text.splitlines():
        line = line.strip()
        match = re.match(r"set_next_step_dir oid=(\d+) dir=(\d+)", line)
        if match:
            direction[int(match.group(1))] = 1 if match.group(2) == "1" else -1
            continue
        match = re.match(
            r"queue_step oid=(\d+) interval=(-?\d+) count=(-?\d+) add=(-?\d+)", line)
        if match:
            oid = int(match.group(1))
            count = int(match.group(3))
            sign = direction.get(oid, 1)
            steps[oid] = steps.get(oid, 0) + sign * count
    return steps


if __name__ == "__main__":
    import json
    import sys
    base = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path("/tmp/opencode/klp1")
    res = run_klipper("G90\nG1 X10 F600\n", base)
    print(json.dumps({k: res[k] for k in ("returncode", "steps", "errors")}, indent=2))
    print(res["mcu_commands"][-1200:])
