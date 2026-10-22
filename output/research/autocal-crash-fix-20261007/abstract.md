# Autocal native crash isolation to the compiler path

Status: investigation incomplete; no verified production fix.

Retrospective summary of saved evidence, written 2026-10-08.

## Abstract

Native crashes were reproduced in objective replay outside the sandbox and in compile-only runs without objective execution. A direct jaxlib replay of saved compiler modules reproduced two segfaults among eleven children without importing JAX or autocal. GDB observed a crash during protobuf object destruction inside the XLA compilation path, but did not locate the first invalid write or free. Small reduced controls passed; they are finite controls rather than fixes. Valgrind timed out with uninitialized-value reports, and the sanitizer build failed to produce a usable wheel. No production fix was established. These observations narrow the necessary trigger set toward native compilation and object lifetime.

## Topics

autocalibration, JAX, jaxlib, XLA, protobuf, SIGSEGV, SIGABRT, compiler lifetime, Valgrind

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Sandbox restrictions are necessary for the crash | Outside-sandbox objective replay | Rejected | [Notebook](research.md) |
| Objective execution or JAX tracing is necessary | Compile-only; direct jaxlib replay | Rejected; 2/11 direct replay children segfaulted | [Batch summary](raw-prefix-three-run/summary.json) |
| Protobuf destruction is the original corruption site | GDB compilation trace | Inconclusive; observed crash site only | [GDB summary](compile-gdb-outside/summary.json) |
| The diagnostic controls establish a production fix | Reduced replay, Memcheck, sanitizer build | Not supported | [Compiler update](../../../research/bug-reports/autocal-native-crashes-october-7-2026/autocal-crash-compiler-update.md) |

## References

- [Isolation decisions and budgets](research.md)
- [Tracked findings and compiler reduction results](../../../research/bug-reports/autocal-native-crashes-october-7-2026/autocal-crash-compiler-update.md)
