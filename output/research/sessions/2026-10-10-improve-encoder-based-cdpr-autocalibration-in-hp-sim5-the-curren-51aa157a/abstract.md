# Faster HP3 encoder autocalibration by reusing settled measurements

Status: complete.

## Abstract

We profiled a supplied HP3 encoder-autocalibration recording, audited candidate anchors against simulated truth, tested sparse recorded datasets, and compared matched physical collectors with known radii and no line buildup. Settling dominated collection time. Raw score disagreed with geometry, and blind thinning often failed. Reusing a completed quiet check reduced 120-point collection from 2,733.214 to 2,571.350 simulated seconds (5.92%), while pooled anchor RMSE stayed 2.11845 mm. A follow-up reproduced the default-force pullout failure exactly through 2,457 encoder readings. Bounding endpoint search, then holding and strictly settling before measurement, recovered collection without relaxing guards. Review of supervised-job commit 383c6d4 exposed unset layering misreported as disabled, plus scene baking and reset settings; both were corrected. A full default-force HP3 job through a socket-denied MCP client then accepted automatically after seven iterations, with 360 points and 2.604 mm anchor RMSE. Its finalized 1.58 GB recording independently contains every measurement and the physical stop boundary. This is simulation evidence from one full default-force trial. Hardware transfer, reliable point-count reduction and score-based geometry selection remain unresolved; the full job is not a paired speed benchmark.

## Topics

autocalibration, HP3, CDPR, encoder-only, collection timing, settling, candidate ranking, observability, sparse sweeps, fixed encoder errors, known radii, no buildup, supervised jobs, MCP, no-layering geometry, reset settings

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Settling is a larger collection cost than encoder averaging or motion waits in the supplied run | E01 | Supported | [Timing and geometry audit](E01-profile-and-geometry.json) |
| Lower raw fit score reliably identifies more accurate anchors | E01, E02, E07–E08 | Rejected | [Original candidates](E01-profile-and-geometry.json), [thinning fits](E02-thinning.json), [matched fits](E08-paired-comparison.json) |
| Uniformly keeping five or six points per direction, or one direction, is a safe default | E02 | Rejected | [Sixteen recorded-data trials](E02-thinning.json) |
| Reusing the quiet check after an unchanged-force move saves time without materially degrading geometry | E03, E05–E08 | Supported in simulation | [Duplicate readings](E03-duplicate-checks.json), [synthetic guards](E05-synthetic-controller-tests.txt), [matched comparison](E08-paired-comparison.json) |
| A single three-sweep span set preserves pooled-fit geometry | E07–E08 | Rejected for these datasets | [Independent and pooled fits](E08-paired-comparison.json) |
| Shorter pullouts can preserve observability | E03 diagnostic only | Not tested | [Travel trace](E03-long-pullouts.json); no altered collection |
| Bounded endpoint search followed by strict measurement settling recovers the reported failure | E11 | Supported in simulation | [Exact prefix](E11-baseline-prefix-comparison.json), [settling result](E11-settling-result.json) |
| Correct no-layering baking permits the default-force supervised workflow to finish with accurate anchors | E12 | Supported in one simulated job | [Acceptance](E12-acceptance.json), [recording audit](E12-rrd-verification.json) |
| Pointwise fixed-encoder geometry, pose elimination or information-per-time acquisition improves calibration | none | Not tested | Proposed in report; no new accuracy result |

| Command sandbox denies localhost sockets while launcher-owned MCP services work | E09 | Supported | [Tooling diagnostic](E09-sandbox-and-tools.json) |

| Networking defaults to enabled while explicit overrides remain available | E10 | Supported, configuration checks only | [Launcher tests](E10-launcher-tests.txt) |

## References

- [Full report](report.md): methods, implementation, measured results and limitations.
- [Research notebook](research.md): objective, budgets, steering and decisions.
- [Session manifest](manifest.json): reference revisions and source/data hashes.
- [Supplied reference](../../../extended-autocal/no-buildup-reference-003/): original simulated collection and candidate history.
- [Matched comparison](E08-paired-comparison.json): fresh baseline/candidate timing, sampling and geometric results.
- [Alexandre dit Sandretto, Daney and Gouttefarde (2012), ROMANSY](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf): encoder redundancy and joint geometry/pose calibration; not an HP3 validation.
- [Maye et al. (2016), DOI 10.1177/0278364915596232](https://journals.sagepub.com/doi/10.1177/0278364915596232): information selection and treatment of unobservable directions.

- [Tooling diagnosis](E09-sandbox-and-tools.json): sandbox EPERM, healthy supervised services and proposed full-auto job mitigation.

- [Three implementation prompts](../../../../research/README.md#local-networking-for-full-auto-runs): scoped full-auto, world-configuration and preflight/test tasks; full-autocal jobs implemented, world-configuration and integration-test tools still proposed.
- [E10 launcher tests](E10-launcher-tests.txt): twelve passing configuration/override checks, not live networking validation.

- [Settling fix and commit review](../../../../research/autocal-settling-review.md): E11 reproduction, P1 correction, checks and E12 limits.
- [Preceding supervised-job report](../2026-10-10-supervised-full-autocal/report.md): retained failure and manual-force lifecycle acceptance; its disabled flags did not prove correct unlayered geometry.
