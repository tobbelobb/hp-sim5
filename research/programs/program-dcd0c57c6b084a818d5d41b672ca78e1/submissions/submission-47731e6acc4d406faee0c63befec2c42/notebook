# Current calibration sensitivity, scale and excitation

Status: complete, awaiting independent review. Assignment `assignment-8512a4d264974d44af743e10bf3071b5`, attempt `attempt-49e6fcc028c24a3b93e352420321ec0d`, program `program-dcd0c57c6b084a818d5d41b672ca78e1`.

## Objective and accepted steering

Measure whether pointwise fixed encoder variation reaches the current finite NumPy/JAX residuals, whether joint geometry/spool scale is ambiguous, and whether more fixed-length excitation or one known radius detects scale. The exact bounded prompt and supplied instructions are retained by the launcher. Initial launch/working notebook text remains in `research-launch-and-working-notes.md`.

User constraints accepted: offline E1/E2/E3 with controls; `.venv/bin/python`; write only this assigned session; no physics, collection, fitting, production edits, commits, new clients or shared dependency/build changes. Parent owns the index, submission and review. The generic index-regeneration instruction does not apply to this worker. A shared index modification was present on entry and was left untouched.

Budget was 18 wall minutes, three experiments, zero fits/collections/physics steps, and 120 seconds per numerical subprocess. All numerical calls were wrapped in `timeout 120s`; no timeout or failed numerical process occurred. The final combined experiments took 2.97 seconds inside the experiment functions. Earlier standalone probes each took under three seconds. Python bytecode was disabled and JAX cache writes stayed in this session. No supervisor action was called.

Later steering accepted: source/output arguments for relocation, compact original inputs and deterministic generator, 1 MiB/file and 8 MiB total recommended archival selection, deterministic gzip plus raw/compressed hashes, a compact summary, stable GitHub source references and fresh-directory reproduction. Archival and documentation work did not rerun numerics.

## Experiments and decisions

| ID | Hypothesis and control | Outcome | Evidence |
| --- | --- | --- | --- |
| E1 | Fixed point associations are discarded; drive inputs and aggregate fixed length must remain active | Supported in isolated residual paths. All 75 synthetic and 60 frozen HP3 residuals per condition/backend finite. Permutation delta exactly zero in both NumPy/JAX; controls changed residuals by 0.16 to 1.13 mm | [E1](E1.json), [summary](summary.json) |
| E2 | Free anchors, radii and dimensionful buildup share a scale symmetry; fixing dimensional quantities should detect scale | Supported conditionally. Clean joint-scale RMS below 1e-12 mm. Fixed buildup and fixed radius controls exceed 1 mm RMS. No universal radius/anchor-only invariance was found | [E2](E2.json.gz) |
| E2 initial discriminator | Co-scaling noise conversion distinguishes an objective normalization effect | Initial comparison inactive under the production 0.05 mm sigma floor. Preserved; no claim drawn from its equality | [Initial result](E2-initial-floor-limited.json.gz), [original script](analysis-v1.py), [original manifest](manifest-E2-initial-floor-limited.json) |
| E2 corrected discriminator | Activate point sigma with explicit 0.001 mm floor, keeping observations unchanged | Fixed-mm-sigma objective scales as s squared; co-scaled-sigma objective constant within 1.8e-11. Unnormalized objective scales as s to the fourth | [E2](E2.json.gz), [summary](summary.json) |
| E3 | Diverse fixed-length encoder sweeps remove scale, compared with duplicate sweeps | Rejected in tested model. Diverse sweeps improve the weakest observable direction 7.51 times, but data rank stays 13/14. Duplicate control preserves normalized singular values | [E3](E3.json) |
| E3 radius control | A single independent radius observation detects the common-scale direction | Supported locally. Assumed 0.05 mm radius uncertainty raises rank to 14/14. A 5% scale change produces 1.96 mm or 39.2 sigma on radius 0 | [E3](E3.json), [summary](summary.json) |

E3 uses both central-difference steps 1e-4 and 1e-5, complete finite Jacobians and explicit coordinate units. No solver was run. E1 reconstructs cost objects for every condition and checks current JAX residual intermediates plus the unmodified production objective. A transformed in-memory AST exposes existing intermediates without source edits.

The main interpretive correction was to separate exact-consistency observation symmetry from nonzero residual magnitudes, noise floors, nominal spool inputs and production priors. Fixed nonzero buildup itself supplies dimensional information. Default buildup and radius-similarity priors can prefer scale separately from observations. The full production objective need not be flat.

## Context and provenance

Read root/research instructions, research README and integration rationale, abstract template, the prior 2026-10-09 encoder-objective audit/report, and the prior validation audit/report in session `fdb616a454af4a6d9c1db074fbdc207d`. Their old trial-004/trial-005 files are absent. Fresh E1 instead uses generated data plus a compact copy of the first three tracked fifth-HP3 reference sweeps. Prior audit reports are context, not required new numerical evidence.

Producer revision is `c0f111c310cc562b7122bf606a59c60de758adbb`. `origin/main` is `7fc26ba491be99c5632e21839be4b3afca201d85`, and `git diff origin/main..HEAD -- autocal` is empty. All referenced autocal files were also compared byte for byte against that commit. GitHub references are pinned there, while producer provenance stays unchanged. [Source references and hashes](source-links.json), [machine/source/input manifest](manifest-all.json), [JAX probe provenance](jax-probe-provenance.json).

`analysis.py` accepts `--repo-root` and `--output`; it does not depend on session directory depth. `inputs.json` holds original compact observations and deterministic seed. The report gives a fresh-directory reproduction command so archived measurements stay intact. Oversized raw JSON is retained; deterministic gzip copies and their two hashes are in [manifest-archive.json](manifest-archive.json). Recommended assets meet both archive limits. Earlier standalone manifests/logs are superseded by `manifest-all.json` for current E1/E2/E3. The preserved initial E2 manifest still labels its now-renamed raw result as `E2.json`.

## Decision handoff

Bounded outcome: `model_gap` supported. Pointwise fixed geometry and independent physical scale information deserve separate attention. Diverse observations help other local directions and do not supply an independent length standard in this model. No hardware accuracy, global uniqueness, fitting improvement or implemented fix is established.

The next cheapest observation would be an offline per-point fixed-length residual prototype and an audit of which dimensional spool/buildup constraints are measured versus nominal. These remain untested. Independent review and broader dynamics/hardware questions belong to the manager. [Full report](report.md), [abstract](abstract.md), [final response](final-message.md).
