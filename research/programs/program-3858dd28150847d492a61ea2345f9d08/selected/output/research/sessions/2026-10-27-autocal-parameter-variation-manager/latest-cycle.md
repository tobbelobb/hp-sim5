# Force-selection persistence and approach decisions

Status: reviewed and published.

The equal60-point coverage variant failed all four exploratory20percent improvement gates, so that variant was cut early. Two fits worsened. Existing automatic force selection was then tested. Two initial force searches hit the per-world step cap after learning settings; they were retained as incomplete runs. Fresh worlds replaying those encoder-derived forces produced useful60-point datasets. Existing restoration recovery reached about3.29mm conventional nominal error; soft fits reached10-12mm. These are limited simulation and reference-dependent findings, not a new force algorithm or full-auto acceptance gain.

The retained production fix atomically saves OUTPUT.force-tuning.json immediately after force selection. It records actual selected forces, diagnostics and machine/spool context and distinguishes successful tuning, fallback and manual settings. It is never automatically loaded. Final dataset fields still reflect any force changes during later recovery. This preserves completed knowledge when a later collection stage fails.

The original Node guard and other reviewed tools were already pushed to origin/codex/autocal-parameter-variation-guards. New source commits39510991 and25822cbb were independently reviewed and pushed. Source controls/behavior tests69cases and Python helper/collector tests17cases passed. The latter627steps are included in the4,166,884step total.

The worker wrote a genuine final message but hit a platform model usage limit before handoff.json. Parent verified deadline cleanup and prepared the selected evidence packet with explicit authorship. No simulation or fitting was restarted after the user continued.

- [Full worker report](../attempt-902d9713dcbe436d8ddc1ba23fd82baa/report.md).
- [Approach ledger](approaches.md).
- [Numerical summary](../attempt-902d9713dcbe436d8ddc1ba23fd82baa/measured-summary.json).
- [Usage and step accounting](../attempt-902d9713dcbe436d8ddc1ba23fd82baa/usage.json).


Final review: review-8b7d4a844d274f31855086c3c4175d5d, supported/keep, code ready, no required checks.
