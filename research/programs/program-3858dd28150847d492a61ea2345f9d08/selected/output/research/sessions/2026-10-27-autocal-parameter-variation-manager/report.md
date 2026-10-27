# HP3 calibration research: preserve force selection and cut weak routes

Status: bounded cycle complete; program paused.

## Abstract

The latest cycle tested a 60-point collection redesign, cut it after it failed the paired improvement gate, and investigated force adequacy instead. Existing encoder-based force selection and recovery produced a nominal fit with 3.29 mm conventional scene-reference error and soft-scene fits around 10 to 12 mm. These are limited simulation results, not demonstrated full-auto or hardware accuracy. Two force searches were interrupted after completing their expensive tuning stage, exposing a production data-retention gap. The retained fix atomically saves selected forces and diagnostics before sweep collection, distinguishes tuning, fallback and manual selection, and never automatically reuses a stale cache. It preserves later force changes in the final dataset. Independent review kept the fix and separate optional per-sweep target support. Sixty-nine JavaScript and seventeen Python checks passed. Earlier Node-runtime and cloud-review fixes remain published. The next unfunded question is reducing force-search cost without degrading collected data.

## Shipped changes

- `25822cbb` preserves force selection before later positioning or collection can fail. `OUTPUT.force-tuning.json` contains selected forces, diagnostics and machine/spool context. Atomic replacement distinguishes tuned success, failed-tuning fallback and manual settings. The final dataset still records forces changed by later recovery. There is no automatic cache reuse.
- `39510991` separately adds optional per-config fixed targets. It preserves anchor/target association and retains distinct levels through deduplication. Existing three-column configs and default acquisition policy are unchanged.
- Earlier published changes include the pinned Node guard and native collection helper, analytical parameter screen, and cloud review-path repair.

The source and reviewed results are published on [codex/autocal-parameter-variation-guards](https://github.com/tobbelobb/hp-sim5/tree/codex/autocal-parameter-variation-guards).

## What the search changed

The tested six-sweep, ten-point layout did not beat the saved three-sweep, twenty-point controls. All four fits failed the predeclared 20 percent exploratory signal; two worsened. That variant was cut early. More independent geometry remains reopenable under different force/data evidence, but it is not receiving more budget now.

Automatic force selection was more useful. Learned-force replay produced complete 60-point datasets. Nominal ordinary LBFGSB hit a sentinel, but the existing restore_points path reached about 3.29 mm conventional error. Soft ordinary fits reached about 10 to 12 mm. This ruled out a new solver policy for the demonstrated case. Saved cross-run prediction covered all 20 nominal and 40 soft points. Unknown encoder-zero pose, changed physical history and only two scenes limit that comparison. No speedup, new force algorithm or full-auto stopping gain is claimed.

The actual production improvement follows directly from a failure. Both initial force searches completed tuning but exhausted their step caps later. Previously the learned configuration was assembled only at the end of all sweeps. The new checkpoint preserves that completed stage before later failures. The source change has lifecycle regression coverage; the native measurements predate it and were not rerun after model-limit interruptions.

## Checks and review

The latest candidate passed 69 behavior/CLI JavaScript checks and 17 Python helper/collector checks, with five slow cases deselected. The 627 regression physics steps are included in the 4,166,884-step cycle total. Research work used six collection invocations and ten fit stages. Measured world wall time totaled about 434 seconds. No physics or fitting was restarted after the final user continuation.

Review `review-8b7d4a844d274f31855086c3c4175d5d` kept fixed submission `submission-72b96ed174494e6286eba68667db11cf`, with code ready and no required checks. Admission `admission-44964abf6b884000a838a6d017c7ae69` applies it. The reviewer inspected the code and selected evidence, independently checked saved arithmetic and holdout reconstruction, and ran no new simulation or fitting.

## Approach decisions

[The approach ledger](approaches.md) distinguishes retired, parked, promising and retained routes. Blanket TRF replacement and uniform thinning are retired within tested scope. Residual-only ranking is parked. The tested equal-point coverage allocation is cut. Existing recovery is kept. Force adequacy and preserving its results are productive. Faster force-search bracketing remains untested and unfunded because a faster search must preserve decisions and data quality.

The original reference-003 raw data is missing, so no direct win over it is claimed. Earlier studies, failed startup trials, partial force collections and genuine worker outputs remain preserved. Platform usage limits interrupted final writing, while owner deadlines still stopped the workers. Manager-created selection manifests are explicitly attributed and do not fabricate actor responses.

## Evidence

- [Latest full research report](../attempt-902d9713dcbe436d8ddc1ba23fd82baa/report.md), [measurements](../attempt-902d9713dcbe436d8ddc1ba23fd82baa/measured-summary.json), [usage](../attempt-902d9713dcbe436d8ddc1ba23fd82baa/usage.json).
- [Exact final review](review-force-checkpoint.json), [approach ledger](approaches.md), [notebook](research.md).
- [Earlier native study](native-report.md), [native review](review-native.json), [first review](review-first.json).
- [Cloud issues and remedies](cloud-environment-issues.json), [literature context](literature.md).

Program: `program-3858dd28150847d492a61ea2345f9d08`. Fixed submissions contain explicit ZIPs of measured inputs/source and failed trials. The program archive records copied files and omissions.
