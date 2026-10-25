# Hangprinter uncertainty program archive

Program `program-1082afc1f40a4e4098c09cc5c68e6564` asked what important things about Hangprinter remain unresolved and which cheapest experiment could change our understanding.

The [research assessment](controller-6664a6157d2e451a8ec92f9b1efd67af/report.md) compares fair validation, measurement-model sensitivity and motor fidelity. Its selected next scientific assignment is a twelve-minute search for compatible common unused whole sweeps, followed conditionally by at most six frozen prediction evaluations. No saved-data scout or new calibration experiment completed. Startup failures reject no scientific hypothesis.

[program-export.json](program-export.json) preserves the existing question, route plans, assignments, steering, failed attempts and review history. The export has zero scientific admissions and retains one pending review obligation. It is a historical snapshot, not a live controller or restored physics world. Historical absolute paths and process identities remain as recorded; they never authorize process adoption or signals. The [research manifest](research-manifest.json) maps the selected evidence to repository paths and hashes.

The [sealed submission](sealed-submission/submission-8dcfa72e3f48459abdf6b894c2932a5d/manifest.json) preserves the original manager-authored startup report and selected evidence, byte for byte. Its separate reviewer failed before assessment. No review verdict was manufactured or applied.

To inspect a restored copy without changing the original store:

```bash
./hp-sim5-research-program --store /tmp/hangprinter-archive import output/research/managers/program-1082afc1f40a4e4098c09cc5c68e6564/program-export.json
./hp-sim5-research-program --store /tmp/hangprinter-archive query program-1082afc1f40a4e4098c09cc5c68e6564 open_questions
```

For a later review, copy the sealed submission directory into the restored store's `submissions/` directory before invoking integrity checks. Do not import this snapshot over the original active store. Resuming actual work requires normal controller recovery and a fresh finite allocation. Runtime connection descriptors, credentials, databases and installed environments are not part of this publication.

Measurements and startup observations were collected at source base `8acde0c21d5b9db8893115e3e79d82d9e41f98ca`, with the explicitly documented local native-build stdio change during later checks. Publication was rebased onto `f3de3dd7ae60f01040b1d08fcece236564dffc0d`. That upstream revision already includes the native fix and the opt-in cloud-client transport. The historical standalone-client failures do not establish a failure of that newer transport. Follow [the current cloud workflow](../../../../scripts/cloud/RESEARCH.md) for future work; no new cloud-transport science was dispatched while preparing these commits.
