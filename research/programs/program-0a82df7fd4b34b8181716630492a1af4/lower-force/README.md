# Lower measurement force, controlled four-attempt partial

Production force default remains5mid. Development max applied error improved3.6959→2.8529mm at1.0641×simulated effort. A nominal candidate encoder transport timeout remains a workflow failure and blocks adoption. Evaluation calibration pair and separate transport experiment are NOT TESTED.

- [Report](report.md), [abstract](abstract.md), [notebook](research.md) and [budget](budget-ledger.json).
- [Exact remaining-job handoff](CONTINUE.md). Only the unrun evaluation pair moves to the next allocation.
- Per-run full raw archives and hash manifests are under n-default/, n-candidate/, d-default/ and d-candidate/. They preserve every event journal, fit, dataset/partial journal and snapshot. Each was readback verified and pushed before the next collection.
- [Compact review bundle](review-evidence.tar.gz) retains all numerical/selection evidence. [Its manifest](review-evidence-manifest.json) explicitly omits only four duplicate events.jsonl files and records their hashes and full durable raw locations.
- [Independent offline recomputation](check_evidence.py) accepts --archive and --manifest. Other mirrored producer scripts are provenance snapshots and require the original session layout.

Fourattempts consumed27fresh fits,7,968,523steps including3003probe steps and470points. Remaining original caps are3collections,53fits,20,031,477steps and1530points. The manager owns fresh wall allocation and any justified seventh transport confirmation.
