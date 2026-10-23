# 001: Review the exact claim using existing evidence first

Benefit: independent criticism without paying for the same simulation again.
Budget: zero simulations, fresh fits or collections. One prompt edit and a short
desk check; no review-of-review loop.

Read [the reviewer](../prompts/reviewer.txt), the cost rules in [README](README.md),
and [Lane 02's attribution correction](../autocal-filtering-lane02-integration.md).

Change `research/prompts/reviewer.txt` to accept an explicit report, candidate
revision/diff and evidence references from the conversation. Find the relevant
work from context if necessary; the latest commit is not necessarily that work.
Keep the existing questions about information gained, confounds, reproducibility,
practical improvement and maintainability, and retain the familiar grade if useful.

The reviewer should inspect the supplied checks and their provenance first,
identify what it verified itself, and request or run an additional check only for
a named material gap. Separate finding quality from code readiness. A negative
finding may be useful; a lower fit score alone may not be an improvement. Produce
one actionable verdict and only useful follow-ups, not three obligatory additions.

Done when:

- The prompt identifies exactly what it reviews and distinguishes reported,
  inspected and independently recomputed evidence.
- It can accept an adequate unchanged measurement without rerunning physics,
  while requiring a relevant missing gate or investigation of a real discrepancy.
- A desk check against Lane 02 explains the individual-fit/consensus mismatch
  from saved evidence; it does not propose recollection to settle attribution.

Verify by reading the revised prompt against that case and a hypothetical changed
physics patch with no stability result. The latter still requires the root
`npm run test:physics` gate. Record this reasoning in progress.txt; no engine or
prompt-grading test framework is needed.
