# Independent review: second HP4 wobble investigation

## Additional findings

This session adds an independent native Rust reproduction and useful limits on
the damping-cache fix. In the frozen HP4 state, legacy one-step/two-pass XPBD
has a 6.272 mm late effector span and 1.787 rad/s peak angular speed. The
first-visit cache reduces that to about `1.2e-10 mm` and zero angular speed.
The result is consistent with the same-backend JavaScript A/B already archived
in the October 19 review, so the physical effect is not specific to the JS
solver.

The cache is not order independent. On the same replay, three passes leave
`0.512 mm` late motion and `0.472 rad/s`; four passes settle, while eight leave
`0.00276 mm` and `0.0069 rad/s`. A global beginning-of-substep pose snapshot
was worse in the tested i2/i4/i8 cases (`0.120-0.268 mm`). These results add a
real caveat to the first-visit approach: it removes the production two-pass
limit cycle in this checkpoint, while other traversal counts can retain small
residual oscillations. The sweep also rejects motor pseudo-stiffness as the
sole cause; removing it reproduced the two-pass cycle. Parameter changes to
cable damping and rotor damping changed the amplitude but did not remove the
feedback mechanism.

The geometry trace strengthens the explanation. At one late step, the two
passes displaced the effector by `0.379 mm` and then `0.285 mm`; the net
two-pass displacement was `0.151 mm`. The iteration sequence is not a
monotone contraction. The late signal autocorrelation peaks at 341 steps
(`0.682 s`, correlation `0.866`), with spectral energy reported over roughly
19-45 Hz. This supports the solver-order feedback diagnosis and shows why
simply interpreting the cycle as too few iterations would be misleading.

## Acceptance status

The longer session still has no completed end-to-end autocal run. Its cleanest
one-substep headless run reached 210,000 steps and entered sweep collection,
then the launcher restarted and killed it before a final manifest. It reused a
pre-tuned force set, so it did not test default-substep `force_max` selection.
Other reported failures predate the deadline correction, and some runs were
stopped or marked contaminated. This is useful progress and failure evidence,
not a successful calibration result.

## Source-level substep claim

The report says a previous one-substep request was implicitly promoted to two
1 ms steps. That claim is not established by the parent source I inspected.
At revision `3c30e6b9`, the JS policy returned the configured count only when it
was greater than one, otherwise it returned one; Python used the same policy.
For valid requested values 1, 2 and 4, the submitted policy change appears to
preserve those counts. Its new `physicsSubstepsEffective` resource and headless
manifest fields are valuable because they make the applied count observable,
but the claimed prior promotion needs a reproducer tied to the older runtime
before treating it as a behavior fix.

## Code review decision

The damping-cache implementation duplicates the already imported fix. The
other candidate changes add effective-substep telemetry, chunk long headless
advances, extend pre-sweep settling, and normalize the configured count. The
telemetry is useful, but the acceptance session ended without a complete run,
and the source-level evidence for an implicit one-to-two promotion conflicts
with its parent implementation. I archived these changes as research evidence
and did not import the extra code. The longer headless bridge waits and the
120-second pre-sweep settling budget also need a completed run to show their
operational cost and benefit.

The agent reports its focused tests and builds passing in `report.md`; I did
not rerun the test suites during this review. RRDs and CSV traces over 1 MiB
were excluded from this archive. The reports, scripts, metric summaries,
small status artifacts and viewer screenshots are preserved alongside this
file.
