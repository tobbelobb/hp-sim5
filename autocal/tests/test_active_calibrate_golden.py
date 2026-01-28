import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GOLDEN = ROOT / "autocal" / "tests" / "data" / "golden_fivesweeps.log"
DATASET = ROOT / "autocal" / "tests" / "data" / "test2_fivesweeps.json"

ANCHOR_LINE_RE = re.compile(r"Anchors:\s*(\[\[.*?\]\])")
NUMBER_RE = re.compile(r"[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?")


def _extract_anchor_numbers(text: str) -> list[float]:
    for line in text.splitlines():
        if "Anchors:" in line:
            match = ANCHOR_LINE_RE.search(line)
            if not match:
                break
            nums = [float(x) for x in NUMBER_RE.findall(match.group(1))]
            if len(nums) == 6:
                return nums
            break
    raise AssertionError("Could not parse 3x2 anchors from output.")


def test_active_calibrate_fivesweeps_golden():
    cmd = [
        sys.executable,
        "autocal/active_calibrate.py",
        "--dataset",
        str(DATASET),
    ]
    result = subprocess.run(
        cmd,
        cwd=ROOT,
        input="q\n",
        text=True,
        capture_output=True,
        check=True,
    )
    stdout = result.stdout
    if stdout.startswith("\n"):
        stdout = stdout[1:]
    if stdout and not stdout.endswith("\n"):
        stdout += "\n"
    normalized = stdout.replace(f"{ROOT}/", "")
    normalized = normalized.replace(str(ROOT), "")
    normalized = re.sub(
        r"Accept anchors \[a\], collect next sweep \[c\], quit \[q\]\?[^\n]*",
        "Accept anchors [a], collect next sweep [c], quit [q]?",
        normalized,
    )
    expected = GOLDEN.read_text(encoding="utf-8")
    actual_nums = _extract_anchor_numbers(normalized)
    expected_nums = _extract_anchor_numbers(expected)
    assert len(actual_nums) == len(expected_nums) == 6
    for idx, (actual_val, expected_val) in enumerate(zip(actual_nums, expected_nums)):
        diff = abs(actual_val - expected_val)
        assert diff <= 100.0, f"Anchor[{idx}] differs by {diff:.3f} (>100.0)"
