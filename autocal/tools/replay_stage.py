"""Replay one calibration stage, with no firmware or collector startup."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import time

from autocal.fit_stage import fit_ellipse_dataset
from autocal.history_selection import evaluate_history_candidates, rank_history_candidates
from autocal.planning_pass import plan_ellipse_sweep
from autocal.stage_artifacts import read_stage_artifact, write_stage_artifact


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("fit", "plan", "evaluate", "rank"))
    parser.add_argument("artifact", type=Path)
    parser.add_argument("--options", type=Path, help="JSON object overriding this stage's saved keyword options")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    artifact = read_stage_artifact(args.artifact)
    payload = artifact["payload"]
    overrides = json.loads(args.options.read_text()) if args.options else {}
    started = time.perf_counter()
    if args.stage in ("fit", "plan"):
        if artifact["kind"] != "fit":
            parser.error("fit and plan require a fit artifact")
        if args.stage == "fit":
            options = {**payload["fit_options"], **overrides}
            # Use the embedded sweep snapshot, never the growing live dataset.
            with tempfile.TemporaryDirectory(prefix="autocal-fit-") as temporary:
                dataset_path = Path(temporary) / "sweeps.json"
                dataset_path.write_text(json.dumps(payload["fit"]["dataset"]))
                options["residuals_csv"] = None
                options["generate_report"] = False
                result = fit_ellipse_dataset(dataset_path, **options)
            output = {**payload, "fit": result, "fit_options": options}
            kind = "fit"
        else:
            options = {**payload["planning_options"], **overrides,
                       "write_cfg": args.output.with_suffix(".cfg.txt"),
                       "collector_output": args.output.with_suffix(".sweeps.json")}
            result = plan_ellipse_sweep(payload["fit"], payload["dataset_path"], **options)
            output = {"plan": result, "planning_options": options}
            kind = "plan"
    else:
        if artifact["kind"] not in ("history", "evaluated_history"):
            parser.error("evaluate and rank require a history artifact")
        if args.stage == "evaluate":
            if artifact["kind"] != "history":
                parser.error("evaluate requires original frozen history candidates")
            options = {**payload["validation_options"], **overrides}
            evaluated = evaluate_history_candidates(payload["candidates"], **options)
            output = {"evaluated": evaluated, "validation_options": options}
            kind = "evaluated_history"
        else:
            if overrides:
                parser.error("rank has no keyword options; change rank_history_candidates to test a policy")
            ranked = rank_history_candidates(payload["evaluated"])
            output = {"ranked": ranked, "chosen_iteration": ranked[0][1]["iteration"] if ranked else None,
                      "failed_predictions": bool(payload["evaluated"]) and not bool(ranked)}
            kind = "ranked_history"
    seconds = time.perf_counter() - started
    output["stage_seconds"] = seconds
    output["input_artifact"] = args.artifact.resolve()
    write_stage_artifact(args.output, kind, output)
    print(json.dumps({"stage": args.stage, "seconds": seconds, "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
