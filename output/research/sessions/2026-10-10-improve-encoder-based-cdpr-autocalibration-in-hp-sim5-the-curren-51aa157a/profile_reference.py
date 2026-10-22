"""Extract encoder collector/Python events and source context from the original RRD."""
import collections
import json
import sys
import time
from pathlib import Path
from rerun.chunk import RrdReader

root = Path(sys.argv[1])
out = Path(__file__).parent
start = time.perf_counter()
kinds = collections.Counter()
examples = {}
n = 0
with (out / "E01-events.jsonl").open("w") as f:
    stream = RrdReader(next(root.glob("*.rrd"))).stream().filter(
        content=["/autocal/python", "/autocal/collector", "/autocal/browser"],
        components="event_json",
    )
    for chunk in stream:
        for cell in chunk.to_record_batch().column("event_json").to_pylist():
            if not cell:
                continue
            e = json.loads(cell[0])
            source, kind = e.get("source"), e.get("kind")
            if source == "browser" and kind != "scene_context":
                continue
            kinds[f"{source}/{kind}"] += 1
            examples.setdefault(f"{source}/{kind}", e)
            if kind == "artifact":
                e = dict(e)
                payload = dict(e["payload"])
                payload["content"] = "<omitted; artifact exists in source run>"
                e["payload"] = payload
            f.write(json.dumps(e) + "\n")
            n += 1
        if n and n % 5000 == 0:
            print(n, round(time.perf_counter() - start, 2), flush=True)
summary = {"kinds": dict(kinds), "examples": examples, "elapsed_s": time.perf_counter()-start}
(out / "E01-event-summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps({"kinds": dict(kinds), "elapsed_s": summary["elapsed_s"]}), flush=True)
