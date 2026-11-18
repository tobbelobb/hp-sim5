# Autocal benchmark history

Accuracy is maximum applied anchor coordinate error in mm. Precision is the largest split-fit anchor coordinate difference in mm. Convergence is simulated seconds to within 2 mm of truth. Total is simulated seconds for the whole run.

Rows group measurements by runtime fingerprint and scoring protocol. Values are medians of successful runs. Convergence requires a majority to report a time. ▲ marks a review threshold crossed since the preceding successful input under the same protocol. Individual results, wall times, and failures are in `results.jsonl`.

## hangprinter_3

| Commit | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| e4785901fa | 2026-11-18T16:53 | 1/1 | 0.96 | 1.22 | 3574 | 5265 | 9 | stable-estimate | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |

## hangprinter_4

| Commit | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| e4785901fa | 2026-11-18T16:53 | 1/1 | 4.42 | 1.31 | unavailable | 5731 | 12 | stable-estimate | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |

## hangprinter_5

| Commit | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| e4785901fa | 2026-11-18T16:53 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |

## hangprinter_5_pulley_reduced

| Commit | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| e4785901fa | 2026-11-18T16:53 | 1/1 | 2.57 | 0.63 | 4133 | 4202 | 6 | good-fit | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |
