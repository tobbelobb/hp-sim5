# HP4 reeved pullout profiles, unscaled size discovery and collection timing

Status: complete — HP3 and HP4 native collections pass; user browser retest still needed.

## Abstract

The user's HP4 browser log009 showed size and sweep endpoint probes stopping at the180-second physical-time limit, followed by an additional0.8 size reduction. We removed that reduction for both machines and centralized stage-specific tuning profiles selected by M666 U on driven axes. Direct HP3 stepping is preserved; reeved HP4 uses larger minimum steps, faster probes, a longer physical budget and less endpoint backoff. Positioning and measurement feeds are doubled for reeved drives. Normal measurement now applies its final preload before the first move and reuses the subsequent quiet check, avoiding a redundant force change and wait. Fresh Rust Native trials completed 24 points on both machines: HP4 ranges1215–1367mm in1221.672physics/281.658wall seconds; HP3 ranges1058–1088mm in746.694physics/21.099wall seconds. A matched HP4 origin-only trial increased cable span from709 to954mm while reducing physics time from270 to218seconds. Collector logs now receive timestamped lines and stage/point wall/physics timing. Browser transport behavior is addressed by a synthetic delay regression, not a live browser benchmark. Hardware strain and calibration accuracy remain unvalidated.

## Topics

HP4, HP3, M666 U, mechanical advantage, reeving, pullout profile, size probe, sweep endpoint, measurement return, raw travel, timestamp, settling, native headless.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| User009 small endpoints were successful geometric detections | User009 log | Rejected: both stop at physical-time limit | [User log](../../../extended-autocal/no-buildup-reference-009/sweeps.full_auto.log) |
| Reeved profile supports full HP4 collection | E36 | Supported:24points, six endpoints stop on response loss, no time-limit exits | [Dataset](E36-hp4-reeved-profile/sweeps.json), [log](E36-hp4-reeved-profile/trial.log) |
| HP4 origin collection gains range with less physics work | E28,E38 | Supported:954vs709mm,218vs270physics seconds | [Matched geometry trial](E38-hp4-profile-point-timing/result.json) |
| Removing size reduction keeps HP3 full collection working | E37 | Supported:24points | [HP3 dataset](E37-hp3-unscaled-direct/sweeps.json) |
| Live-clock request overhead can cause tiny-step short timeouts | Synthetic RPC regression | Supported in model; actual browser timing not measured | [Regression source](../../../../autocal/control/tests/behaviors/controlled_pullout.test.mjs) |
| Full browser user009 behavior is fixed or calibrated accuracy improves | None | Not tested | No live browser retest or calibration solve |

## References

- Implementation revision `d9620fd8`: profiles, unscaled size discovery, preload reuse and timestamped collection logs.

- [Report](report.md), [notebook](research.md), [numerical results](reeved-profile-results.json).
- [Pullout glossary and experiment guide](../../../../autocal/control/behaviors/PULLOUTS.md), [profile knobs](../../../../autocal/control/behaviors/pullout_profiles.mjs): stage meanings, units and M666 U selection.
- [E36 HP4 result](E36-hp4-reeved-profile/result.json), [E37 HP3 result](E37-hp3-unscaled-direct/result.json), [E38 timing log](E38-hp4-profile-point-timing/trial.log). Trial manifests retain scene/native artifact configuration; source diffs and profile-v1.mjs retain tested settings.
- [Preceding endpoint evaluation](encoder-v1-report.md): original failures preserved, not superseded silently.

## The terms and code changes

1. **Preload probe:** small increasing-force motion near the starting state establishes useful preload; return home. It does not find an edge.
2. **Size probe:** paired-drive motion estimates usable offsets for fixed anchors by reading a different, free encoder. Its logged driven travel and measured free-line travel are different quantities. The latter is now used directly; it was already measured after probe backoff, so the extra0.8 shrink was a second reduction.
3. **Fixed-anchor positioning:** set the lengths that remain fixed during a sweep, translating the sweep arc.
4. **Sweep endpoint probe:** one driven line explores the local endpoint, with the sensor line under preload. This is the sideways pullout.
5. **Measurement return:** inward position moves and encoder samples produce calibration data.
6. **Return home:** restore encoder reference lengths between experiments/on request. Probe backoff is a separate short inward move to the selected interior endpoint.

The user-observed up/back/side sequence is consistent with these stages, but the short log cannot reconstruct exact Cartesian motion. No recording was invented. The detailed glossary and all stage-specific knobs live beside the behavior code.

M666 U is mechanical advantage. Profiles inspect driven axes only: U1 selects direct (HP3); U2 low anchors selects reeved (HP4). A held U4 top motor alone does not select the reeved policy. Existing mmPerDeg and force conversions already account for U; there is no second multiplication of those conversions.

| Probe setting | Direct / HP3 | Reeved / HP4 |
| --- | --- | --- |
| Nominal / minimum cable step |10 /0.5mm|25 /5mm|
| Probe feed |1200mm/min|3600mm/min|
| Response-ratio stop |5|10|
| Physical-time budget |180s|480s|
| Interior backoff |90%|95%|
| Positioning / measurement feed scale |1x configured feed|2x configured feed|

The common travel/step bounds, tracking/fixed-anchor guards and net-response plateau check remain. No M569.8 or load constraints are used. The change is stronger position exploration, not a new guessed edge-force number. Remove measuredMaxTravelScale and its conversion function rather than keeping a no-op deprecated path.

Native headless physics advances explicitly; browser physics continues while requests are in flight. The same physical-time budget can therefore be consumed by many small network-paced steps. User009 proves the time-limit stop, not the transport diagnosis. A synthetic regression advances physics one second per RPC and uses identical linear sensor response: the direct tiny-step policy times out under200mm, while the reeved policy reaches a 500 mm requested interval (475mm after backoff). This isolates a possible timing failure without pretending to reproduce the user's browser.

## Measurements

| Trial | Points | Physics seconds | Observed wall seconds | Canonical cable spans (mm) |
| --- | --- | --- | --- | --- |
|E34-hp3-net-response-bootstrap|24|686.644|18.897|1005.96, 1020.08, 995.81|
|E37-hp3-unscaled-direct|24|746.694|21.099|1071.31, 1087.66, 1057.79|
|E28-hp4-local-origin|8|269.946|54.011|708.71|
|E38-hp4-profile-point-timing|8|218.358|42.066|953.67|
|E36-hp4-reeved-profile|24|1221.672|281.658|1215.35, 1366.69, 1338.45|

E36 full HP4 succeeds where the preceding local endpoint attempt E35 stopped at21/24points after360wall seconds. That is a completion comparison, not a percentage speedup against a completed workload. The earlier continuous-force E23 completed 24 points in936.396physics/216.394wall seconds but sampled much smaller540/555/492mm spans. Full reeved collection is wider, not universally faster.

E28/E38 origin-only trials share fixed target lengths0,0 and eight points. E38 uses about19.1% fewer physics seconds with34.6% greater canonical cable span. Observed wall time also decreases, but concurrent workload varied; this is not a controlled backend throughput benchmark. E34/E37 HP3 comparison changes fixed offsets through removal of the0.8 reduction: spans increase about6–7%, while physics work increases about8.7%. HP3 stepping and acceptance criteria are unchanged.

The new native size probe measured470.147mm unscaled. User009 measured224.155mm before its0.8 conversion, but browser/native initial-state and timing differences prevent treating those as a matched multiplier. Canonical spans are cable-coordinate ranges, not Cartesian workspace fractions.

## Timing and waits

Every collector line written through the full-auto parent gets an ISO8601 timestamp with timezone plus elapsed wall seconds. Child stdout/stderr are streamed through the same writer in browser and headless modes. Internal phase and point logs report wall and physical-clock seconds. This distinguishes size/endpoint search, preload settling, point settling and total measurement-return time. Python solver output redirected separately can still contain unprefixed lines; the collector timing path is timestamped.

Previously the first normal measurement applied a2x preload and settled, moved and settled, then changed to5x measurement preload and settled again. It now applies the final measurement preload before moving; post-move stability is reused for sampling. Zero-tension projection retains its distinct force sequence. Sample counts, encoder-noise statistics, drift checks and stability tolerances were not relaxed. Some settling remains necessary and can dominate near large excursions; E38 logs expose its actual duration rather than hiding it.

## Validation and limitations

127 JavaScript behavior/primitive/CLI tests and41 Python CLI/replay tests pass. Tests include profile selection by actual driven-axis U, unchanged HP3 settings, unrelated held reeving, live-clock RPC delay, first-point preload reuse, drift rejection and legacy/new replay. All new simulated data uses Rust Native at requested 10x (HP3) /25x (HP4), fixed 2 ms steps, closed-loop motors, no layering and no RRD. Maximum trial budgets360wall/1800physical seconds for full HP4,240wall/1500physical for smaller trials. Supervisor preflight remained source-stale; isolated fresh worlds and a successful command-context socket probe were used.

No live browser experiment, hardware test, load replay or calibration solve was performed. Successful collection does not certify line strain or calibrated geometry. E36/E37 include movement and preload changes; E38 additionally records detailed settle phases. Later additions only improve timing/size metadata and tests, without changing the tested movement policy. All jobs have finished; no further experiments are pending.
