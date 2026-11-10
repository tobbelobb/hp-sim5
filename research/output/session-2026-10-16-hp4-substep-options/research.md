# Substep options and size-tune timeout

Accepted steering: expose existing substeps beside browser/headless motor settings; consider a simple larger timeout if motion is a genuine torque response. Retain default one and unchanged stability acceptance.

Budget: bounded offline native fixed-torque pulls at one/four steps, no physics recording; production size-tune helper against actual native world; targeted UI/settings/integration tests. No full-autocal or hardware run selected this turn.

E01: 600 s fresh-world pulls, then extend one-step to 2400 s to distinguish finite decay from perpetual motion. Evidence in pull-*.json, scripts retained. The extended one-step run overwrote pull-1.json; original 600 s output preserved as pull-1-600.json.
E02: unchanged settling function over raw physical samples rejects one-step at 120 s and accepts at 472 s; four-step accepts at 120 s. Sampled at 1 s; not exact original trajectory.
E03: actual production size-tune routine against native physical world with 500 ms polling succeeds after 477.75 s. Set size-only budget600 s and finite wall630 s. No settling threshold changes.
E04: option controls, invalid values, acknowledgements and headless reset persistence tested. Browser visible count1->4->1, stopped. 169 JS tests, 2 settings/reset integration tests,25 additional headless checks passed. Remaining concern: timestep-dependent numerical dynamics, not a reason to claim full calibration validated.

No active experiment remains. No commits. Report and abstract record limitations.


## Follow-up: slow half-sweep pullout in reference-006

User requested only a timeout increase to retry. The actual reference-006/sweeps.full_auto_1.log shows successful size tuning (590.418 mm raw), then first half-sweep pullout hitting its 120 s limit, holding the drive, and timing out after the subsequent 30 s preload-settling limit. Increased both pullout travel and its immediate preload-settling budget to 600 simulated seconds, each with a 630 s wall deadline. Force selection, substep defaults and stability criteria are unchanged. No new physical trial or full-autocal retry was run. The two targeted settling/size-tune suites pass: 13 tests. This supersedes the earlier report statement that other force-travel limits were unchanged.
