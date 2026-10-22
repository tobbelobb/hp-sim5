# Flag inventory and literal usage evidence

Literal option spellings, file counts not invocations; includes help/comments. Absence is not non-use. Defaults/JSON keys not counted. Files >8 MB excluded.

Revision `e7179384192a9a06bda3b548b16aaa94d864e5dc`. Scanned 13863 files. See [raw evidence](usage.json) and [audit script](audit_usage.py).

Definitions can include the variant parser, whose `None` means inherit. The first definition is generally the main parser; consult its source before interpreting defaults.

| Flag | User KEEP | Definition / default expression | Log files | Other files | Example |
| --- | --- | --- | ---: | ---: | --- |
| `--b-bounds` | KEEP | `autocal/_autocal_common.py:3409`: `None`; `autocal/_autocal_common.py:4078`: `None` | 1 | 9 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3401` |
| `--b-prior-sigma` |  | `autocal/_autocal_common.py:3421`: `None`; `autocal/_autocal_common.py:4080`: `None` | 2 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3413` |
| `--base-radii` | KEEP | `autocal/_autocal_common.py:3367`: `None`; `autocal/_autocal_common.py:4075`: `None` | 4054 | 46 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--buildup-factor` | KEEP | `autocal/_autocal_common.py:3373`: `None`; `autocal/_autocal_common.py:4076`: `None` | 4069 | 51 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--candidate-count` |  | `autocal/_autocal_common.py:3578`: `41` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3569` |
| `--candidate-deltas` |  | `autocal/_autocal_common.py:3577`: `None` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3568` |
| `--collector-args` | KEEP | `autocal/_autocal_common.py:3731`: `()` | 78 | 25 | `autocal/data/default_dataset.full_auto.log:1` |
| `--config` | KEEP | `autocal/_autocal_common.py:3743`: `None` | 5 | 12 | `autocal/data/klipper_1.full_auto_1.log:6` |
| `--dataset` | KEEP | `autocal/_autocal_common.py:3820`: `None` | 4140 | 90 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--debug-sweep-actions` |  | `autocal/_autocal_common.py:3785`: `(argparse implicit)` | 5 | 10 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3773` |
| `--delta-max` |  | `autocal/_autocal_common.py:3580`: `None` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3571` |
| `--delta-min` |  | `autocal/_autocal_common.py:3579`: `None` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3570` |
| `--existing-tol-mm` |  | `autocal/_autocal_common.py:3584`: `10.0` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3576` |
| `--extended-recording` | KEEP | `autocal/autocal.py:1770`: `(argparse implicit)` | 0 | 3 | `autocal/data/references/extended-reference.md:10` |
| `--extended-reference` |  | `autocal/autocal.py:1774`: `(argparse implicit)` | 17 | 27 | `autocal/data/references/extended-reference.md:55` |
| `--extended-reference-ws` |  | `autocal/autocal.py:1772`: `(argparse implicit)` | 6 | 10 | `autocal/data/references/extended-reference.md:30` |
| `--fd-eps-mm` |  | `autocal/_autocal_common.py:3581`: `1.0` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3572` |
| `--filter-schedule` |  | `autocal/_autocal_common.py:3460`: `'warmup,warmup,warmup,dynamic'`; `autocal/_autocal_common.py:4086`: `None` | 410 | 11 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_01cbb9aa/five_points_bigger_deltas.full_auto.log:1` |
| `--find-buildup-factor` | KEEP | `autocal/_autocal_common.py:3359`: `'off'`; `autocal/_autocal_common.py:4068`: `None` | 62 | 34 | `autocal/data/fifth_hp3_dataset.full_auto_89.log:1` |
| `--find-radii` | KEEP | `autocal/_autocal_common.py:3351`: `'off'`; `autocal/_autocal_common.py:4061`: `None` | 4093 | 65 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--firmware` | KEEP | `autocal/_autocal_common.py:3737`: `'rrf'` | 1368 | 15 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:107` |
| `--fit-structure` |  | `autocal/_autocal_common.py:3451`: `'3'`; `autocal/_autocal_common.py:4085`: `None` | 1154 | 7 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_000b9760/five_points_bigger_deltas.full_auto.log:1` |
| `--flex` |  | `autocal/_autocal_common.py:3350`: `(argparse implicit)`; `autocal/_autocal_common.py:4055`: `None` | 3 | 10 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3341` |
| `--full-auto-log` |  | `autocal/_autocal_common.py:3855`: `None` | 3620 | 42 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--full-auto-run` |  | `autocal/_autocal_common.py:3840`: `None` | 3 | 3 | `autocal/data/layered_manual_some_bad_points.full_auto.log:1` |
| `--headless-sim` | KEEP | `autocal/autocal.py:1778`: `(argparse implicit)` | 61 | 27 | `autocal/data/references/extended-reference.md:29` |
| `--help` | KEEP |  | 1 | 11 | `output/research/fifth-hp3-reference-audit/current/autocal/README_elliptical_feature_calibration.md:121` |
| `--hp-sim-reset` | KEEP | `autocal/_autocal_common.py:3775`: `(argparse implicit)` | 4055 | 8 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:3` |
| `--keep-sim-alive` | KEEP | `autocal/_autocal_common.py:3770`: `(argparse implicit)` | 1 | 7 | `autocal/data/references/extended-reference.md:83` |
| `--klipper-config` | KEEP | `autocal/_autocal_common.py:3758`: `None` | 1 | 2 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3736` |
| `--line-width` | KEEP | `autocal/_autocal_common.py:3379`: `DEFAULT_LAYER_LINE_WIDTH_MM`; `autocal/_autocal_common.py:4088`: `None` | 11 | 12 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_5191e62f/five_points_bigger_deltas.full_auto.log:1` |
| `--low-anchor-z` |  | `autocal/_autocal_common.py:3397`: `None`; `autocal/_autocal_common.py:4091`: `None` | 20 | 6 | `autocal/data/fifth_hp3_dataset.full_auto_1.log:1` |
| `--machine-type` | KEEP | `autocal/_autocal_common.py:3804`: `(argparse implicit)` | 4162 | 76 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--max-steps` |  | `autocal/_autocal_common.py:3837`: `20` | 82 | 24 | `autocal/data/default_dataset.full_auto_9.log:1` |
| `--min-fixed-delta-spacing-mm` |  | `autocal/_autocal_common.py:3590`: `20.0` | 8 | 3 | `autocal/data/old_data/ten_points_bigger_deltas4.full_auto.log:1` |
| `--no-collect` | KEEP | `autocal/_autocal_common.py:3810`: `(argparse implicit)` | 4138 | 72 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--no-exclude-existing` |  | `autocal/_autocal_common.py:3583`: `(argparse implicit)` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3574` |
| `--no-pointwise-filtering` |  | `autocal/_autocal_common.py:3493`: `(argparse implicit)`; `autocal/_autocal_common.py:4007`: `(argparse implicit)` | 35 | 11 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_39eee9ab/five_points_bigger_deltas.full_auto.log:1` |
| `--no-robust-debug` |  | `autocal/_autocal_common.py:3554`: `(argparse implicit)` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3546` |
| `--no-sweep-wise-filtering` |  | `autocal/_autocal_common.py:3517`: `(argparse implicit)`; `autocal/_autocal_common.py:4029`: `(argparse implicit)` | 36 | 11 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_1327cfb9/five_points_bigger_deltas.full_auto.log:1` |
| `--no-trf-solver-fallback` |  |  | 12 | 4 | `autocal/data/.regress_parallel_runs/23_april_2026_df588e62/23_april_2026.full_auto.log:1` |
| `--objective-schedule` |  | `autocal/_autocal_common.py:3470`: `'1,1,1,1'`; `autocal/_autocal_common.py:4087`: `None` | 27 | 7 | `autocal/data/fifth_hp3_dataset.full_auto_39.log:1` |
| `--open-loop-motors` | KEEP | `autocal/_autocal_common.py:3729`: `(argparse implicit)` | 0 | 1 | `autocal/tests/test_autocal_cli.py:373` |
| `--optimizer-mode` |  | `autocal/_autocal_common.py:3338`: `_normalize_optimizer_mode(os.environ.get('AUTOCAL_OPTIMIZER_MODE', 'fast'))`; `autocal/_autocal_common.py:4097`: `None` | 1726 | 16 | `autocal/data/.regress_parallel_runs/fifth_hp3_dataset_c79a9cd7/fifth_hp3_dataset.full_auto.log:1` |
| `--output-with-explanations` |  | `autocal/_autocal_common.py:3793`: `(argparse implicit)` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3781` |
| `--patience` |  | `autocal/_autocal_common.py:3866`: `3` | 2 | 9 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3850` |
| `--physics-substeps` | KEEP | `autocal/_autocal_common.py:3727`: `(argparse implicit)` | 32 | 13 | `output/research/index.md:1509` |
| `--plot-residual-histogram` |  | `autocal/_autocal_common.py:3832`: `(argparse implicit)` | 2 | 6 | `output/research/fifth-hp3-reference-audit/current/autocal/README_elliptical_feature_calibration.md:136` |
| `--pointwise-filtering` |  | `autocal/_autocal_common.py:3487`: `(argparse implicit)`; `autocal/_autocal_common.py:4001`: `None` | 1 | 11 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3479` |
| `--pointwise-global-mad` |  | `autocal/_autocal_common.py:3499`: `(argparse implicit)`; `autocal/_autocal_common.py:4012`: `None` | 2 | 11 | `autocal/data/fifth_hp3_dataset.full_auto_37.log:1` |
| `--pointwise-per-sweep-mad` |  | `autocal/_autocal_common.py:3505`: `(argparse implicit)`; `autocal/_autocal_common.py:4018`: `(argparse implicit)` | 1 | 11 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3497` |
| `--pointwise-residual` |  | `autocal/_autocal_common.py:3481`: `'sampson'`; `autocal/_autocal_common.py:3996`: `None` | 4 | 6 | `autocal/data/fifth_hp3_dataset.full_auto_35.log:1` |
| `--project-zero-tension` |  | `autocal/_autocal_common.py:3780`: `(argparse implicit)` | 13 | 14 | `autocal/data/old_data/flexible_lines_2000_project_zero_tension.full_auto.log:1` |
| `--r0-bounds` | KEEP | `autocal/_autocal_common.py:3403`: `None`; `autocal/_autocal_common.py:4077`: `None` | 810 | 34 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--r0-prior-sigma-mm` |  | `autocal/_autocal_common.py:3415`: `None`; `autocal/_autocal_common.py:4079`: `None` | 11 | 3 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_635e73f2/five_points_bigger_deltas.full_auto.log:1` |
| `--record` | KEEP | `autocal/autocal.py:1770`: `(argparse implicit)` | 1 | 5 | `autocal/data/references/extended-reference.md:10` |
| `--recording` | KEEP | `autocal/autocal.py:1770`: `(argparse implicit)` | 10 | 8 | `autocal/data/references/extended-reference.md:10` |
| `--recording-mode` | KEEP | `autocal/autocal.py:1776`: `(argparse implicit)` | 44 | 11 | `autocal/data/references/extended-reference.md:45` |
| `--regularization` |  | `autocal/_autocal_common.py:3582`: `1e-06` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3573` |
| `--report` |  | `autocal/_autocal_common.py:3566`: `(argparse implicit)` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3557` |
| `--residuals-csv` |  | `autocal/_autocal_common.py:3560`: `None` | 1 | 13 | `output/research/fifth-hp3-reference-audit/current/autocal/README.md:76` |
| `--robust-debug` |  | `autocal/_autocal_common.py:3548`: `(argparse implicit)` | 1 | 6 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3540` |
| `--rrf-config` | KEEP | `autocal/_autocal_common.py:3752`: `None` | 27 | 7 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3735` |
| `--scale-fix` |  | `autocal/_autocal_common.py:3445`: `'3'`; `autocal/_autocal_common.py:4084`: `None` | 1607 | 7 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_000b9760/five_points_bigger_deltas.full_auto.log:1` |
| `--shotgun` |  | `autocal/_autocal_common.py:3846`: `(argparse implicit)` | 10 | 7 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_b22277e0/five_points_bigger_deltas.full_auto.log:1` |
| `--sigma-floor-mm` |  | `autocal/_autocal_common.py:3385`: `None`; `autocal/_autocal_common.py:4089`: `None` | 1 | 12 | `autocal/tests/test_autocal_cli.py:44` |
| `--sigma-source` |  | `autocal/_autocal_common.py:3529`: `'auto'`; `autocal/_autocal_common.py:4039`: `None` | 21 | 14 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_b22277e0/five_points_bigger_deltas.full_auto.log:12` |
| `--sigma-used-mm` |  | `autocal/_autocal_common.py:3391`: `None`; `autocal/_autocal_common.py:4090`: `None` | 1 | 12 | `autocal/tests/test_autocal_cli.py:46` |
| `--sim` | KEEP | `autocal/_autocal_common.py:3764`: `(argparse implicit)` | 4323 | 77 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--simulation` | KEEP | `autocal/_autocal_common.py:3764`: `(argparse implicit)` | 0 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:454` |
| `--solve-iterations` |  | `autocal/_autocal_common.py:3322`: `400`; `autocal/_autocal_common.py:4095`: `None` | 43 | 17 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_0fbf049e/five_points_bigger_deltas.full_auto.log:1` |
| `--solve-optimizer` |  | `autocal/_autocal_common.py:3323`: `'lbfgsb'`; `autocal/_autocal_common.py:4096`: `None` | 128 | 28 | `autocal/data/.regress_parallel_runs/23_april_2026_20ed5acd/23_april_2026.full_auto.log:1` |
| `--solve-restarts` |  | `autocal/_autocal_common.py:3321`: `4`; `autocal/_autocal_common.py:4094`: `None` | 119 | 8 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_0fbf049e/five_points_bigger_deltas.full_auto.log:1` |
| `--sparse-recovery` |  | `autocal/_autocal_common.py:3815`: `(argparse implicit)` | 13 | 12 | `autocal/data/.regress_parallel_runs/23_april_2026_8627928d/23_april_2026.full_auto.log:1` |
| `--speedup` | KEEP | `autocal/_autocal_common.py:3882`: `None` | 223 | 35 | `autocal/data/apr_23.full_auto_1.log:1` |
| `--spool-inner-iters` |  | `autocal/_autocal_common.py:3433`: `30`; `autocal/_autocal_common.py:4082`: `None` | 40 | 7 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_20b1ca0b/five_points_bigger_deltas.full_auto.log:1` |
| `--spool-outer-iters` |  | `autocal/_autocal_common.py:3427`: `3`; `autocal/_autocal_common.py:4081`: `None` | 46 | 7 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_4f02e802/five_points_bigger_deltas.full_auto.log:1` |
| `--spring-k-multiplier` |  | `autocal/_autocal_common.py:3349`: `1.0`; `autocal/_autocal_common.py:4092`: `None` | 2 | 15 | `autocal/tests/test_calibrate.py:292` |
| `--stage-artifacts` |  | `autocal/_autocal_common.py:3851`: `None` | 56 | 33 | `autocal/data/references/extended-reference.md:157` |
| `--stop-cost` |  | `autocal/_autocal_common.py:3838`: `None` | 2 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3825` |
| `--stop-std-mm` |  | `autocal/_autocal_common.py:3839`: `None` | 2 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3826` |
| `--sweep-metric` |  | `autocal/_autocal_common.py:3523`: `'outlier_ratio'`; `autocal/_autocal_common.py:4034`: `None` | 22 | 14 | `autocal/data/.regress_parallel_runs/five_points_bigger_deltas_b22277e0/five_points_bigger_deltas.full_auto.log:12` |
| `--sweep-points` |  | `autocal/_autocal_common.py:3826`: `None` | 28 | 15 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3814` |
| `--sweep-wise-filtering` |  | `autocal/_autocal_common.py:3511`: `(argparse implicit)`; `autocal/_autocal_common.py:4023`: `None` | 1 | 11 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3503` |
| `--theta0-mode` |  | `autocal/_autocal_common.py:3439`: `'zero'`; `autocal/_autocal_common.py:4083`: `None` | 4 | 7 | `autocal/data/old_data/five_points_bigger_deltas.full_auto_10.log:1` |
| `--threshold` |  | `autocal/_autocal_common.py:3348`: `250.0`; `autocal/_autocal_common.py:4093`: `None` | 1 | 25 | `autocal/tests/test_calibrate.py:290` |
| `--top-k` |  | `autocal/_autocal_common.py:3596`: `10` | 1 | 3 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3587` |
| `--trf-solver-fallback` |  | `autocal/_autocal_common.py:3329`: `False` | 0 | 4 | `autocal/tests/test_autocal_cli.py:195` |
| `--use-noise-mean` |  | `autocal/_autocal_common.py:3536`: `(argparse implicit)`; `autocal/_autocal_common.py:4044`: `None` | 1 | 11 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3528` |
| `--use-raw-lengths` |  | `autocal/_autocal_common.py:3542`: `(argparse implicit)`; `autocal/_autocal_common.py:4050`: `(argparse implicit)` | 2 | 11 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3534` |
| `--verbose` |  | `autocal/_autocal_common.py:3861`: `(argparse implicit)` | 754 | 28 | `autocal/data/.regress_parallel_runs/23_april_2026_046947ab/23_april_2026.full_auto.log:1` |
| `--write-sweep-config` |  | `autocal/_autocal_common.py:3875`: `None` | 2 | 6 | `output/research/fifth-hp3-reference-audit/current/autocal/_autocal_common.py:3859` |
