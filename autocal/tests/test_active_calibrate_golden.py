import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GOLDEN = ROOT / "autocal" / "golden_fivesweeps.log"
DATASET = ROOT / "autocal" / "data" / "test2_fivesweeps.json"


def test_active_calibrate_fivesweeps_golden():
    cmd_display = (
        "python autocal/active_calibrate.py ellipse-loop --work-dataset autocal/data/test2_fivesweeps.json"
    )
    cmd = [
        sys.executable,
        "autocal/active_calibrate.py",
        "ellipse-loop",
        "--work-dataset",
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
    actual = f"$ {cmd_display}\n\n{normalized}"
    expected = GOLDEN.read_text(encoding="utf-8")
    assert actual == expected
