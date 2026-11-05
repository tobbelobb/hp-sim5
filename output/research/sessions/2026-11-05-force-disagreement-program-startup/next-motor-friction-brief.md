# Commissioned lower-measurement-force iteration

Assignment: assignment-a64d3f287c374d4e8b0e697ee33afe68. Status: commissioned, not yet dispatched; no numerical results yet.

The earlier motor-friction-only proposal is superseded by an implementation-focused paired test: current default sensor force min(forceMax,5*forceMid) versus fixed candidate min(forceMax,4*forceMid). Pullout and preload forces, guards, solver, stopping and selection stay unchanged. Prior independent force-valley evidence motivates four, with no tuning on new truth.

{
  "machine": "hp3",
  "radii": "known 30 mm",
  "buildup": 0,
  "layering": false,
  "guides_mu": 0,
  "candidate": "sensor measurement force min(forceMax,4*forceMid); all pullout/preload/settling/stopping/solver rules unchanged",
  "worlds": "nominal noise .03 seed4001; dev motor factors [.5,2,1.5,1]/[2,.5,1,1.5] sigma .03 seed4101; eval [.5,1.5,2,.75]/[1.5,2,.5,1] sigma .05 seed4201",
  "failure": "any anchor >20 mm or no completed calibration",
  "good": "every anchor <5 mm",
  "effort_limit": "1.20 times paired completed baseline simulated time/steps"
}

Budget: 100 worker + 15 review + 10 manager/startup minutes, 190 seconds cleanup. At most seven collections, eighty fits, 28M steps, 2000 points; per job 4M steps and 8000 sim seconds. Sequential native Rust, speedup25, recording none plus full journals and passive snapshots. Preflight and separate command socket probe required. Reserve last ten worker minutes for report, artifacts and cleanup. Commit and push safe checkpoints after every collection. No automatic main-branch merge.
