"""Measure unchanged full regression batteries with bounded outer scheduling."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import gc
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import resource
import runpy
import signal
import subprocess
import sys
import threading
import time

import psutil

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
RUNNER = ROOT / "autocal/tools/regress_calibration_logs.py"


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def child(workers, output):
    namespace = runpy.run_path(str(RUNNER), run_name="schedule_benchmark")
    runner_globals = namespace["main"].__globals__
    original_run = runner_globals["run_autocal"]
    lock = threading.Lock()
    start = time.monotonic()
    active = peak_active = 0
    rows = []

    def observed_run(**kwargs):
        nonlocal active, peak_active
        spec = kwargs["dataset_spec"]
        dataset = kwargs["dataset_path"]
        row = {"dataset": spec.name, "fixture_sha256": hashlib.sha256(dataset.read_bytes()).hexdigest(),
               "scratch_dir": str(dataset.parent), "start_s": time.monotonic() - start}
        with lock:
            active += 1
            peak_active = max(peak_active, active)
        try:
            returncode, text = original_run(**kwargs)
            row.update(returncode=returncode, end_s=time.monotonic() - start)
            row["elapsed_s"] = row["end_s"] - row["start_s"]
            (output / f"{spec.name}.txt").write_text(text)
            with lock:
                rows.append(row)
                with (output / "dataset-events.jsonl").open("a") as log:
                    log.write(json.dumps(row) + "\n")
            print("BENCHMARK_DATASET " + json.dumps(row), flush=True)
            return returncode, text
        finally:
            with lock:
                active -= 1

    class BoundedExecutor(ThreadPoolExecutor):
        def __init__(self, max_workers=None, **kwargs):
            effective = min(max_workers or workers, workers)
            print(f"BENCHMARK_EXECUTOR requested={max_workers} effective={effective}", flush=True)
            super().__init__(max_workers=effective, **kwargs)

    runner_globals["run_autocal"] = observed_run
    runner_globals["concurrent"].futures.ThreadPoolExecutor = BoundedExecutor
    sys.argv = [str(RUNNER), "--no-fail-score-mismatch", "--keep-going", "--color", "never"]
    print(f"BENCHMARK_START workers={workers} gc_enabled={gc.isenabled()}", flush=True)
    returncode = namespace["main"]()
    write_json(output / "datasets.json", {"workers": workers, "peak_active": peak_active,
               "gc_enabled": gc.isenabled(), "datasets": sorted(rows, key=lambda r: r["dataset"])})
    return returncode


def run_battery(workers, index):
    output = HERE / f"{index:02d}-workers-{workers}"
    output.mkdir(exist_ok=False)
    command = [sys.executable, str(Path(__file__).resolve()), "--child", "--workers", str(workers), "--output", str(output)]
    env = os.environ.copy()
    env["PYTHONFAULTHANDLER"] = "1"
    for name in ("PYTHONPATH", "LD_PRELOAD", "PYTHONMALLOC"):
        env.pop(name, None)
    cpu_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    started = datetime.now(timezone.utc).isoformat()
    start = time.monotonic()
    peaks = {"calibration_processes": 0, "native_threads": 0, "aggregate_rss_bytes": 0}
    timed_out = False
    with (output / "runner.txt").open("w") as log:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        root_process = psutil.Process(process.pid)
        while process.poll() is None:
            processes = []
            try:
                processes = root_process.children(recursive=True)
            except psutil.NoSuchProcess:
                pass
            calibrations = threads = rss = 0
            for current in processes:
                try:
                    if any(Path(arg).name == "autocal.py" for arg in current.cmdline()):
                        calibrations += 1
                    threads += current.num_threads()
                    rss += current.memory_info().rss
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            peaks["calibration_processes"] = max(peaks["calibration_processes"], calibrations)
            peaks["native_threads"] = max(peaks["native_threads"], threads)
            peaks["aggregate_rss_bytes"] = max(peaks["aggregate_rss_bytes"], rss)
            if time.monotonic() - start > 1800:
                timed_out = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                break
            time.sleep(0.5)
        returncode = process.wait()
    elapsed = time.monotonic() - start
    cpu_after = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu_s = cpu_after.ru_utime + cpu_after.ru_stime - cpu_before.ru_utime - cpu_before.ru_stime
    datasets = json.loads((output / "datasets.json").read_text()) if (output / "datasets.json").exists() else None
    text = (output / "runner.txt").read_text()
    row = {"run": index, "workers": workers, "started_utc": started, "elapsed_s": elapsed,
           "cpu_s": cpu_s, "average_cpu_cores": cpu_s / elapsed, "returncode": returncode,
           "timed_out": timed_out, "sampled_peaks": peaks, "output_dir": str(output),
           "dataset_attempts": len(datasets["datasets"]) if datasets else None,
           "native_failures": [r for r in datasets["datasets"] if r["returncode"] in (-6, -11)] if datasets else None,
           "other_child_failures": [r for r in datasets["datasets"] if r["returncode"] not in (0, -6, -11)] if datasets else None,
           "generated_logs": text.count("Generated log: "), "all_datasets_pass": "ALL DATASETS: PASS" in text,
           "exact_summaries": text.count("SUMMARY: EXACTLY EQUAL")}
    write_json(output / "summary.json", row)
    print(json.dumps(row), flush=True)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--workers", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.child:
        return child(args.workers, args.output)
    fixtures = ROOT / "autocal/data/references"
    write_json(HERE / "provenance.json", {"git_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
               "python": sys.version, "interpreter": sys.executable,
               "versions": {p: version(p) for p in ("jax", "jaxlib", "numpy", "scipy", "psutil")},
               "runner_sha256": hashlib.sha256(RUNNER.read_bytes()).hexdigest(),
               "fixtures": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(fixtures.glob("*.json"))},
               "environment": {k: os.environ.get(k) for k in ("XLA_FLAGS", "JAX_CPU_ENABLE_ASYNC_DISPATCH", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS")},
               "cpu_affinity": sorted(os.sched_getaffinity(0)), "order": [11, 2, 2, 11]})
    rows = []
    for index, workers in enumerate((11, 2, 2, 11), 1):
        row = run_battery(workers, index)
        rows.append(row)
        write_json(HERE / "summary.json", rows)
        if row["timed_out"]:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
