# Evidence in this directory

| file | experiment | content |
| --- | --- | --- |
| `refset_solver_compare.{txt,json}` | E28 | canonical replay of the 11 legacy fixtures, `lbfgsb` vs `trf` |
| `pool_evidence_clean.txt`, `pool_evidence.json` | E30/E32 | clean 22-pool benchmark (11 target no-buildup HP3 recordings + 11 legacy fixtures) x {lbfgsb, trf}: applied/oracle/rule RMSE |
| `union_rule_clean.txt` | E33 | candidate-pool union of both solvers |
| `thin_study_boundary.json` | E34 | point decimation N = 10..16 on 10 target recordings |
| `harness_crosstalk.txt` | E31 | why the first staging harness (`/tmp/arts_a|b`) was discarded |
| `offline_span.txt`, `pool_robustness.txt` | superseded | computed from the contaminated `/tmp/arts_a|b` pools; kept only as a record of the harness bug |

Tools that produced these: `../tools/stage_clean_pools.py`, `../tools/pool_evidence.py`,
`../tools/union_rule.py`, `../tools/thin_study.py`, `../tools/refset_solver_compare.py`,
`../tools/conc_probe.py`, `../tools/harness_probe.py`, `../tools/cmd_probe.py`.
