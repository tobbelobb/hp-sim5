# Cloud Research Program audit

Status: complete within tested scope; final production regression interrupted.

Question: Does the managed cloud environment support the full current Research Program smoothly, and which measured fixes or limits remain?

Program: program-7aa80aa18f10499e85cb24ffeae712cb.

The human authorized extensive simulation and compute, allowing about two hours. The manager starts with a single bounded worker and a separate fresh reviewer, then chooses physics, collection and lifecycle follow-ups from their evidence. No hardware accuracy claim is in scope.

Competing explanations: stale dependencies or inherited command permissions account for failures, or implementation defects persist after prerequisites pass.

Initial observations: startup completed; the first runtime check found a stale native addon and denied socket creation. A command with the supported network permission passed socket creation, loopback bind and local HTTP. The first controller was shut down through its tool. Native addon rebuild completed in 2.8 seconds. The same program reopened with network and Git metadata write permission. All runtime readiness checks then passed. No worker attempt was spent on setup retries.

Budgets: four distinct tasks within a finite 100 minute allocation, 10 minutes of review per task, 190 seconds cleanup. First assignment allows 22 minutes, no fresh fits or collections, at most 20,000 direct physics steps, plus the named isolated test suites with their own bounded timeouts. Numerical counters are contracts, not hard aggregate metering.

Next decision: inspect real worker tool coverage and separate review, then commission the smallest missing production simulation or lifecycle test. Preserve failures and report untested paths.

## Baseline and fixes

Baseline worker attempt-ca404fcaf323469eb35cc37b515ca3a7 completed with launcher_finally_completed cleanup. Its default command socket probe returned EPERM. A network-permission-bearing worker command did not return before intervention at 341.6 seconds, with unknown approval/execution outcome. Worker-authored MCP scripts were then executed in the manager's explicitly permitted context. This is parent-assisted evidence and does not establish a smooth original actor attachment.

HP3 preflight f2b3dcc8b52644d1a9cbd9bf3746df8d passed in 0.8184 seconds with the attached world unchanged at step zero. Worker readback inspected all six data chunks of the isolated preflight RRD. Research-services job ce13fb80cdbd4bbcac28f0fbd234768f had 172 passes and three failures. Two failures came from the scrubbed test environment losing the writable Warp cache; the third came from treating the launcher's shared output symlink as an escaped input path. Both requested fresh 500-step experiments failed before run creation, so direct baseline physics is zero steps. Headless-autocal job 46d4d37e13c541a9bf8e8d403c121f66 passed 49 tests.

The first baseline review launch failed before actor dispatch. The cloud bridge rejected the default store's .research-programs/reviews final-output path. No assessment was admitted. Its status is preserved in baseline-review-first-status.json.

Source commit 3c5cc88b fixes the approved experiment output root, assigns test-owned Warp/XDG caches, passes each configured cloud session directory explicitly, and adds a launcher-owned file relay to the existing real MCP attachments. This relay exposes only hp_sim5 and the assigned Rerun server. It adds no model client, shell runner or physics engine. New regression tests exercise actual MCP calls while the requester is forbidden to create sockets, expired/stale requests, bounded unknown-outcome timeouts and hard bridge-owner loss. Focused output/cache checks passed 88 tests. Cloud/relay/owner checks passed 35 tests. Live authenticated actor use of the relay remains pending.

The initial 100 minute allocation was ended after baseline cleanup. The controller shut down normally and reopened the same program with the new source and permitted launcher context. A separately funded ten minute correction of the frozen baseline review is running with a fresh real reviewer. No history or store was discarded. Next allocation will target the remaining production tool categories and real actor file-relay use.

The container uses tail as PID 1 and retained 209 zombie processes after destructive lifecycle fixtures. These are exited processes, not live physics services. The cgroup reports no PID limit. This is a container init/reaping limitation and remains separate from tool-owned live-process cleanup.

## Current production evidence

The corrected baseline review became review-302fad39e77d4b9e9475e86d7ab2b55f with keep/supported/report_only admission-7cdcc17edfaf415a88e16e50bfbbf1ca. The original failed review launch remains retained.

The first production launch failed before an actor existed because the CPU image had no configured graphics ICD for Rerun. Parent probes showed Chromium's installed SwiftShader Vulkan driver works. A second protocol issue surfaced because the pinned Rust Viewer MCP exits on the Python SDK's modern server/discover request. Source 379318b2 selects software rendering on GPU-free cloud images while preserving explicit driver choices, reloads that setting for direct Python launch, uses initialize for the pinned Viewer MCP, and keeps Viewer MCP settings/cache in the assigned session. A real Viewer/file-relay regression and 34 focused tests passed before the next dispatch.

Actor attempt-e13302e362f545e39608027dcb85f963 then used the actual relay successfully while its own command sockets remained denied. It invoked all 23 advertised tools. research-services job 419ce7d0faf74c2abf76be2a9fb3018f passed 176 tests. A 500-step recorded/unrecorded native pair produced byte-identical 501-row telemetry with SHA256 f9ca994cb413bf4a08398ff5a05ee61712e555c83be99a3439b0b58f2ffeaa4e. Six collection points validated, a second collection stopped at its fixed boundary, and configuration, reset, firmware, Viewer and a real browser page worked. Headless Chromium startup used the manager's permitted context, while the actor performed browser operations through its own relay. The page had no errors. Resets retired the first browser URL; that stale connection attempt was preserved and the exact replacement page was used.

The production trial found two more defects. browser_action interventions returned a list through a dictionary MCP schema. Full-auto job d5f69c419e474b54b098a2077715349d completed 1,305,459 steps/2610.918 simulated seconds with 24 point-journal entries and a finalized 96,931,452-byte events RRD, but a raw Infinity best_score_ui made strict status JSON fail. The same failure prevented terminal job.json persistence and clearing active ownership, blocking subsequent jobs/recovery. Budget stop is not automatic calibration acceptance. Report and raw data remain scoped to379318b2.

Source 7c0a353e wraps browser history in an interventions object, reports nonfinite status diagnostics as null plus explicit value markers, clears terminal job ownership before persistence, and keeps HTTP encoding errors within the error-response boundary. Raw optimizer evidence stays unchanged. 125 targeted tests passed, including Infinity/-Infinity/NaN progress, terminal persistence, subsequent job dispatch, actual MCP output validation and usable HTTP error responses.

The production worker ended with launcher_finally_completed. All 27 observed descendant identities, including the owned Chromium session, were no longer live after cleanup. Its frozen partial-readiness submission is under separate review. A distinct fresh-source regression will repeat the exact bounded full-auto settings, verify terminal status and slot reuse/cancellation, read browser history on a real page, and retain a compact RRD without repeating broad numerical work.

## Wrap-up after interruption

The human requested commits and a push for future Research Programs. On continuation the cloud runtime had been recreated, no actual subagents remained, and the final attempt had recorded wall expiry and launcher_finally_completed cleanup. Its initial HP3 preflight and browser-service startup are preserved. The parent browser command stalled in a platform permission request and was eventually aborted. No final worker report, terminal full-auto or subsequent cancellation evidence was available. This attempt is inconclusive. No actor response or bridge completion was invented.

The same program reopened in a permitted controller context and retained historical attempts, allocations and admissions, with zero reserved workers/reviewers and no unknown cleanup IDs. No physics world started. Final combined tests at runtime source 7c0a353e passed 333 tests, one skipped, eight deselected and 34 subtests in 125.70 seconds, including the actual optional Viewer test. Finish fixed code/evidence review, save the scoped archive, then publish the fixes through a branch and pull request. GPU/hardware accuracy, advanced live pools and final production status/history lifecycle remain outside the established result.

Final independent review `review-95277ad5c83e42a9bcee4b5a0aa8499b` verified all 17 selected asset hashes, parsed the 333-test/34-subtest result and inspected the fixed source diff. Admission `admission-7338901984c24981a30b0280df87943d` is keep/supported with code readiness ready for the bounded repair scope. It explicitly retains the final production regression as inconclusive. [Review](final-review.json), [admission](final-admission.json). No new physics was run by the reviewer.
