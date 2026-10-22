# Autocal native crash investigation

Objective: locate the first native memory error behind intermittent autocal
SIGSEGV/SIGABRT and verify a small fix if the evidence identifies one.

Starting evidence: research/bug-reports/autocal-native-crashes-october-7-2026/
and /tmp/autocal-crash-investigation-20261007/. Original dependencies retained:
Python 3.12.3, NumPy 2.4.3, SciPy 1.17.1, JAX/jaxlib 0.9.2.

Hypothesis: the captured objective replay can expose corruption before the
later GC/NumPy fault. No dependency upgrade or GC workaround accepted as a fix.

Constraints: offline replay only; no collection or robot motion. Use .venv/bin/python.
Initial compute budget: three uninstrumented 11-process objective replay batches,
one bounded 10-minute Valgrind replay; adapt after inspecting evidence. Full
calibration validation only after a concrete candidate fix, with declared counts.

Results: nohook-1 lost one of eleven processes; debug-allocator lost five of
eleven; NumPy control passed 2.2 million iterations. Outside-sandbox ordinary
replay also lost one of eleven, excluding the sandbox as a necessary trigger.
Valgrind outside reaches objective cases, but roughly 100–160 seconds per case;
its 600-second budget remains in force. See the tracked follow-up report.

Next isolation experiment: eleven compile-only children, Python debug allocator,
each capped at 600 seconds. Input signatures use ShapeDtypeStruct; no JAX arrays
are created and no compiled objective kernel executes. This tests whether
runtime kernels/input-buffer lifetime are necessary for corruption. Stop
expanding experiments until this distinction and the Valgrind outcome are known.

Compile-only already reproduced SIGABRT (`corrupted double-linked list`) after
13 signatures, without any JAX data arrays or kernel executions. Next two
bounded controls (11 children each, 600 seconds per child): lower-only with five
cycles, excluding backend compilation; and compile-only with
XLA_FLAGS=--xla_cpu_parallel_codegen_split_count=1. The helper now uses a per-case
function; record this lifetime difference from the original compile-only loop
if comparing failure incidence. These are diagnostic controls, not production
changes or accepted mitigations.

## Continued investigation, October 7 after 14:50 CEST

Accepted steering: keep hunting until the native crash is fixed and verified.
Previous assessment yielded a completed GDB trace, so that turn was progress.
Current environment remains Python 3.12.3, JAX/jaxlib 0.9.2, NumPy 2.4.3.
No production fix accepted. No robot motion or collection.

GDB finished: ten children completed, child 5 segfaulted during third signature
098f0547c7e1a38d in protobuf InternalMetadata::DeleteOutOfLineHelper, reached
through FrontendAttributes/HloInstructionProto/HloModuleProto destruction and
PjRtCpuClient::CompileAndAssignDevices. This is a crash site, not the first write.

Next bounded experiments: prefix of three signatures, eleven workers and ten
cycles (330 compilations); then the third signature alone with the same worker
count and cycles (110 compilations). Each child capped at 600 seconds. Keep
normal GC, debug Python allocator, original source and default XLA settings.
Decision: whether the compiler crash survives a greatly reduced signature set;
then isolate JAX tracing from compilation with a saved compiler IR module.

Prefix-three replay: all eleven completed ten cycles, 330 compilations total.
Single third-signature replay: all eleven completed ten cycles, 110 compilations.
Neither finite control reproduced the failure; no fix inferred.
Captured compiler modules/options before backend_compile_and_load for the three
signatures. Direct jaxlib compilation of the third module passed a smoke check
without importing JAX or autocal. Next budget: eleven direct jaxlib children,
fifty cycles of three modules, 600-second cap each (1,650 compilations maximum).
This removes objective imports, Python tracing and JAX cache-clearing from the
reproduction; it deliberately changes lifetimes, so success is only a control.

User steering at approximately 15:13 CEST: just write findings to another
Markdown file. No new experiment launched after this steering. Created
research/bug-reports/autocal-native-crashes-october-7-2026/autocal-crash-compiler-update.md.
Direct jaxlib replay reproduced two SIGSEGVs among eleven children without JAX
or autocal imports. Memcheck timed out after three complete compilations, with
uninitialized-value reports in protobuf (six contexts), no invalid-access
report located. Sanitizer build ended unsuccessfully with terminated Clang
actions; no wheel produced. Both tool sessions confirmed terminal.
Root cause/fix still unverified. See the new report for exact counts and paths.
