# Autocal benchmark history

Accuracy is maximum applied anchor coordinate error in mm. Precision is the largest split-fit anchor coordinate difference in mm. Convergence is simulated seconds to within 2 mm of truth. Total is simulated seconds for the whole run.

Rows group measurements by runtime fingerprint and scoring protocol. Values are medians of successful runs. Convergence requires a majority to report a time. Individual results, wall times, and failures are in `results.jsonl`.

## hangprinter_3

| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| 20da983feb | v2 5cb6a2120b678a5e1b10 | 2026-11-18T18:17 | 1/1 | 0.96 | 2.27 | 3574 | 4547 | 7 | stable-estimate | ok | Merge pull request #109 from tobbelobb/codex/autocal-benchmark-history-review |
| e4785901fa | v1 21a14da94822c449f3df | 2026-11-18T16:53 | 1/1 | 0.96 | 1.22 | 3574 | 5265 | 9 | stable-estimate | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |

## hangprinter_4

| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| 20da983feb | v2 fdc9c52ee787dbdde295 | 2026-11-18T18:17 | 1/1 | 5.75 | 0.94 | unavailable | 4693 | 8 | stable-estimate | ok | Merge pull request #109 from tobbelobb/codex/autocal-benchmark-history-review |
| e4785901fa | v1 db83d3f94b9573bc9a39 | 2026-11-18T16:53 | 1/1 | 4.42 | 1.31 | unavailable | 5731 | 12 | stable-estimate | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |

## hangprinter_5

| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| 20da983feb | v2 c9af9758cc9c363ae274 | 2026-11-18T18:17 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok | Merge pull request #109 from tobbelobb/codex/autocal-benchmark-history-review |
| e4785901fa | v1 ec76c752d8af462b4731 | 2026-11-18T16:53 | 1/1 | 0.83 | 0.69 | 3424 | 3768 | 5 | good-fit | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |

## hangprinter_5_pulley_reduced

| Commit | Fingerprint | Date | OK / runs | Accuracy | Precision | Converged | Total | Sweeps | Stop | Status | Subject |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|
| 20da983feb | v2 1a2005fcdd1766204c2c | 2026-11-18T18:17 | 1/1 | 2.57 | 0.63 | 4133 | 4202 | 6 | good-fit | ok | Merge pull request #109 from tobbelobb/codex/autocal-benchmark-history-review |
| e4785901fa | v1 cd2405c0977da2d289d1 | 2026-11-18T16:53 | 1/1 | 2.57 | 0.63 | 4133 | 4202 | 6 | good-fit | ok | Merge pull request #108 from tobbelobb/codex/autocal-benchmark-ci-history |
