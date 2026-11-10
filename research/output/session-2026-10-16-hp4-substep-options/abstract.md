# HP4 substep options and bounded size-tuning travel

Substeps are now selectable in the browser and through autocal's --physics-substeps setting for browser/headless collection, defaulting to one. A fixed-torque no-layering HP4 experiment shows the unchanged settling check passing after about 472 simulated seconds at one step versus 120 at four steps. Running the production size-tuning routine against native physics with its real ramp and 500 ms polling passed at one step after 477.75 simulated seconds. The size-tuning deadline is increased from 120 to 600 seconds, with a finite independent wall deadline and unchanged settling criteria. This validates the size-tuning routine, not an entire full-autocal run or hardware accuracy. Timestep-dependent numerical damping remains an accuracy concern.

Topics: HP4, autocal, settling, optional substeps, headless settings, numerical damping.

See [report](report.md).


## Follow-up: slow half-sweep pullout in reference-006

User requested only a timeout increase to retry. The actual reference-006/sweeps.full_auto_1.log shows successful size tuning (590.418 mm raw), then first half-sweep pullout hitting its 120 s limit, holding the drive, and timing out after the subsequent 30 s preload-settling limit. Increased both pullout travel and its immediate preload-settling budget to 600 simulated seconds, each with a 630 s wall deadline. Force selection, substep defaults and stability criteria are unchanged. No new physical trial or full-autocal retry was run. The two targeted settling/size-tune suites pass: 13 tests. This supersedes the earlier report statement that other force-travel limits were unchanged.
