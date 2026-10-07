# Autocal native crash: compiler isolation update

Written October 7, 2026, approximately 15:14 CEST (UTC+02:00).
This continues [autocal-crash-followup.md](autocal-crash-followup.md).
**No production fix has been established.** Calibration source and the project
`.venv` dependencies remain unchanged.

## Strongest new finding

**The failure reproduces by compiling saved compiler inputs directly through
jaxlib, without importing either JAX or autocal.** Two of eleven independent
processes segfaulted. This removes the objective's Python code, JAX tracing,
SciPy optimization, and JAX cache-clearing from the necessary trigger set.
No compiled objective kernel was executed.

The first invalid write/free and the faulty native component remain unknown.
The evidence now supports investigating the jaxlib/XLA compiler and native
object lifetime path directly. It does not establish that protobuf itself is
the original cause.

## Completed GDB result from the previous investigation

The follow-up report was last modified at 14:43:09 CEST and says GDB was still
running. Its saved summary finished at **14:43:59 CEST**. Ten children completed
all 56 signatures; child 5 failed after approximately 9.6 seconds. GDB reported
SIGSEGV while compiling the third signature, `098f0547c7e1a38d`, after the first
two signatures had compiled successfully.

The fault was on a native worker thread, inside
`.venv/lib/python3.12/site-packages/jaxlib/libjax_common.so`:

```text
google::protobuf::internal::InternalMetadata::DeleteOutOfLineHelper<UnknownFieldSet>
xla::FrontendAttributes::~FrontendAttributes
xla::HloInstructionProto::SharedDtor
xla::HloInstructionProto::~HloInstructionProto
google::protobuf::internal::RepeatedPtrFieldBase::DestroyProtos
xla::HloComputationProto::~HloComputationProto
...
xla::HloModuleProto::~HloModuleProto
xla::PjRtCpuClient::CompileAndAssignDevices
xla::PjRtCpuClient::CompileAndLoad
```

The fault address was `0x26`. The recorded instruction dereferenced offset 8
from register `r14`, whose value was `0x1e`. This is a concrete invalid-pointer
observation during protobuf object destruction, not evidence of where that
pointer first became invalid.

Evidence: [native trace](../../../output/research/autocal-crash-fix-20261007/compile-gdb-outside/05.txt)
and [completed summary](../../../output/research/autocal-crash-fix-20261007/compile-gdb-outside/summary.json).

## New reduction experiments

These experiments retained Python 3.12.3, NumPy 2.4.3 and JAX/jaxlib 0.9.2.
The current objective hash still matches the historical baseline:

```text
b4a7979205e079ab304522bc5e11fc5088b37fe734e986266caebb8d53803792
```

Each batch used eleven subprocesses, normal cyclic GC and
`PYTHONMALLOC=debug`. Each child had a 600-second limit. The batch launchers
set the original single-thread BLAS/OpenMP environment variables and removed
`PYTHONPATH` and `LD_PRELOAD`. No XLA-flag change was introduced.

| Experiment | Work per child | Result |
| --- | --- | --- |
| JAX replay of the first three signatures | 10 cycles, 30 compilations | All 11 completed; 330 compilations total |
| JAX replay of signature `098f0547c7e1a38d` alone | 10 cycles, 10 compilations | All 11 completed; 110 compilations total |
| Direct jaxlib replay of three saved modules | Up to 50 cycles, 150 compilations | Two SIGSEGVs; nine children completed |

The passing reductions are finite controls, not fixes. The direct jaxlib batch
used more cycles and changed the input representation and object lifetimes;
its failure rate cannot be compared as though only one factor changed.

### How the direct compiler replay works

A diagnostic script intercepts `jax._src.compiler.backend_compile_and_load`
before compilation and saves each module's textual MLIR plus serialized
`CompileOptions`. It captured these three signatures:

```text
003a106feb4e7470
064d42f9827e9ded
098f0547c7e1a38d
```

A separate script imports `jaxlib.xla_client`, creates a CPU client, reads
those saved inputs, and repeatedly calls:

```python
compiled = client.compile_and_load(
    module_bytes,
    client.local_devices(),
    xc.CompileOptions.ParseFromString(options_bytes),
)
del compiled
gc.collect()
```

The child startup records confirm `jax_imported=false` and
`autocal_imported=false`. The script creates no JAX data arrays and never
executes an executable. NumPy is still imported through jaxlib; this is not a
NumPy-free control.

Child 5 segfaulted after approximately 114.36 seconds, while compiling
`003a106feb4e7470` in cycle 22, after 66 completed compilations. Child 10
segfaulted after approximately 114.25 seconds, while compiling
`098f0547c7e1a38d` in cycle 21, after 65 completed compilations. Cycle numbers
are zero-based. The nine surviving children completed 150 compilations each:
**1,481 completed compilations across the batch**, followed by two interrupted
compilations. Both failed children returned `-11`; none timed out.

Evidence: [batch summary](../../../output/research/autocal-crash-fix-20261007/raw-prefix-three-run/summary.json),
[child 5](../../../output/research/autocal-crash-fix-20261007/raw-prefix-three-run/05.txt),
[child 10](../../../output/research/autocal-crash-fix-20261007/raw-prefix-three-run/10.txt).
These failures have Python fault-handler output, but no new native backtrace;
their exact native fault sites have not been confirmed identical to the GDB fault.

## Native memory diagnostics

### Valgrind: new reports, incomplete replay

A direct jaxlib replay of `098f0547c7e1a38d` alone ran under Valgrind Memcheck
with `PYTHONMALLOC=malloc`, origin tracking and a 600-second limit. It completed
three compilations, started a fourth, then timed out with command exit 124.
It did not complete the requested five cycles.

Valgrind reported **605,651 errors from six contexts**, all recorded as
conditional branches or moves depending on uninitialized values. The stacks
include protobuf descriptor construction and reflection:

```text
google::protobuf::DescriptorBuilder::BuildFieldOrExtension
google::protobuf::Reflection::ListFields
google::protobuf::util::MessageDifferencer::Compare
xla::HloInstruction::CreateFromProto
xla::HloModule::CreateFromProto
xla::ConvertStablehloToHloWithOptions
```

Reported origins include heap allocation in protobuf's descriptor flat
allocator. No `Invalid read`, `Invalid write` or `Invalid free` report was
found in this capture. These uninitialized-value reports are new diagnostic
leads; their relationship to the segfaults has not been established. The
timeout is not a passing result.

Evidence: [Memcheck log](../../../output/research/autocal-crash-fix-20261007/raw-memcheck.txt)
and [replay output](../../../output/research/autocal-crash-fix-20261007/raw-memcheck-stdout.txt).

### AddressSanitizer build: unsuccessful

The matching JAX `jax-v0.9.2` source was downloaded into
`/tmp/autocal-jax-0.9.2-source`, at commit
`a659757d768587a81d095a9fab5f0c36f8beb218`. Its pinned XLA commit is
`187a5eb58277a85847d1516bd1e20b7faf03d5ef`.

An isolated jaxlib build used `--config=asan`, six build jobs and a Bazel
output root under `/tmp/autocal-jax-bazel`. It failed after approximately
548 seconds. The log reports terminated Clang compilation actions for LLVM
and protobuf; the reason for termination was not established. **No usable
sanitized wheel was produced or tested.** Nothing was installed into `.venv`.

Evidence: [build log](../../../output/research/autocal-crash-fix-20261007/asan-build.txt).
Both the build and the bounded Valgrind command were confirmed terminal before
this report was written; neither diagnostic is still running.

## Evidence location and next decision

New working artifacts are under the ignored directory
`output/research/autocal-crash-fix-20261007/`:

- `export_compiler_input.py`: capture compiler modules and options.
- `raw_compile.py`, `run_raw_compile.py`: direct compiler replay and batch runner.
- Three `<signature>.mlir` and `<signature>.options` pairs: frozen compiler inputs.
- `prefix-three.json`, `gdb-failing-case.json`: reduced argument-signature sets.
- `prefix-three-run/`, `single-case-run/`, `raw-prefix-three-run/`: completed batches.
- `raw-memcheck.txt`, `raw-memcheck-stdout.txt`, `asan-build.txt`: diagnostics.

These ignored artifacts and the `/tmp` source/build trees may be absent in
another checkout. This Markdown preserves the findings, but the direct
compiler reproducer is not yet packaged into tracked evidence.

The next useful action is to preserve that reproducer and obtain native memory
diagnostics on it: investigate the sanitizer build termination, then run an
instrumented matching compiler. A native backtrace from the direct jaxlib
failure would also test whether it reaches the same destruction path as the
earlier GDB crash. Treat the protobuf uninitialized-value reports as a separate
lead until a causal connection is demonstrated.

Do not accept a dependency upgrade, GC change or concurrency limit as a
root-cause fix from these results. A candidate fix still needs repeated compiler
and objective replay, followed by complete 11-dataset calibration batteries
with normal GC and numerical reference validation.
