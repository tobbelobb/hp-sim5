"""E11: matched-ness and time comparison of the two 3-sweep arms (tolerance 0.01 vs 0.05).

Usage: .venv/bin/python e11_compare.py
Reads artifacts/e11/arm[AB]_*_{3sweeps,manifest}.json and prints per-sweep
setpoint/length agreement plus the simulated-time and step-count difference.
"""
import json
from pathlib import Path

S = Path(__file__).resolve().parents[1]
E11 = S / "e11"


def load(tag):
    d = json.load(open(E11 / f"{tag}_3sweeps.json"))
    return [(tuple(s["fixed_anchors"]), s["drive_anchor"], s["sensor_anchor"], s["data_points"])
            for s in d["sweeps"]]


def main():
    A, B = load("armA_tol0.01"), load("armB_tol0.05")
    for i, (a, b) in enumerate(zip(A, B)):
        same_cfg = a[:3] == b[:3]
        spA = [round(float(x["drive_setpoint_mm"]), 3) for x in a[3]]
        spB = [round(float(x["drive_setpoint_mm"]), 3) for x in b[3]]
        dsp = max(abs(x - y) for x, y in zip(spA, spB))
        dl = {f: max(abs(x[f] - y[f]) for x, y in zip(a[3], b[3]))
              for f in ("l_drive", "l_sensor")}
        print(f"sweep{i} cfg={a[:3]} cfg_match={same_cfg} max|dsetpoint|={dsp:.4f}mm "
              f"max|dl_drive|={dl['l_drive']:.4f} max|dl_sensor|={dl['l_sensor']:.4f} "
              f"span A={spA[-1]-spA[0]:.3f} B={spB[-1]-spB[0]:.3f}")
    ma = json.load(open(E11 / "armA_tol0.01_manifest.json"))
    mb = json.load(open(E11 / "armB_tol0.05_manifest.json"))
    sa, sb = ma["simulated_s"], mb["simulated_s"]
    na, nb = ma["steps_executed"], mb["steps_executed"]
    print(f"\nsimulated_s A={sa} B={sb} delta={sa-sb:.3f} ({(sa-sb)/sa*100:.2f}% of A)")
    print(f"steps       A={na} B={nb} delta={na-nb} ({(na-nb)/na*100:.2f}%)")
    print(f"per point   {(sa-sb)/ma['point_count']:.3f} s over {ma['point_count']} points")
    extra_mm = 90.4780 - 47.0350
    print(f"arm A sweep 2 covers {extra_mm:.3f}mm more travel = {extra_mm/1400*60:.2f}s at feed 1400")


if __name__ == "__main__":
    main()
