# Autocal crash follow-up — October 7, 2026

Status: investigation in progress. **No production fix established.** The
repository calibration code and `.venv` dependencies have not been changed.

## New results

1. **Confirmed objective-only reproduction on the original JAX/jaxlib 0.9.2,
   without the earlier custom native signal handler.** Eleven independent
   processes replayed 56 captured objective inputs, with 100 evaluations per
   input and explicit garbage collections. Process 6 died with SIGSEGV after
   approximately 179 seconds, during `gc.collect()` inside the evaluation loop.
   The other ten processes each completed all 5,600 evaluations. This removes
   the custom signal handler as a necessary trigger. SciPy optimization and the
   autocal planning/control loop were not involved.

2. **Python's debug allocator did not prevent the failure.** The same
   eleven-process replay with `PYTHONMALLOC=debug` lost five processes: three
   SIGSEGVs and two SIGABRTs. Fault sites include native compilation, explicit
   garbage collection, and final `jax.clear_caches()`. The aborts reported
   `corrupted double-linked list` and `corrupted size vs. prev_size`. There was
   no Python debug-allocation padding diagnostic identifying the first write.
   This is additional evidence of native allocator corruption, not a fix.

3. **A NumPy-only control passed.** Eleven processes each completed 200,000
   repetitions of `np.linalg.lstsq(...)[0].astype(float, copy=False)`, checking
   finite results and collecting garbage every 100 repetitions: 2.2 million
   repetitions total. No JAX import or objective evaluation was involved.
   This control does not exonerate NumPy, but the previously reported cast
   crash site is insufficient to identify NumPy as the cause.

4. **Valgrind encountered a separate diagnostic failure before NumPy loaded.**
   A sandboxed replay using `PYTHONMALLOC=malloc`, disabled debuginfod, fair
   thread scheduling and affinity to two CPUs crashed during Python import
   handling. Valgrind reported a jump to invalid address `0x1004b3d6a0`, reached
   through `mbstowcs` / `PyUnicode_DecodeLocale` / filesystem error handling.
   The script had printed `before numpy`; it had not printed `before objective`.
   Do not attribute that failure to the autocal objective. Another sandboxed
   Valgrind attempt exhausted its 600-second limit without reaching an
   objective-result marker or reporting an invalid access; that is not a pass.

5. **The objective replay also fails outside the sandbox.** The approved
   eleven-process replay lost process 5 to SIGSEGV after approximately 167
   seconds; the other ten completed all 5,600 evaluations. Thus removing the
   sandbox is not a fix and the sandbox is not a necessary trigger. The
   outside-sandbox Valgrind probe imported NumPy and the objective and
   completed four captured cases before its 600-second timeout. It reported
   zero invalid accesses in the work it reached; it did not complete the
   corpus and is not evidence that the objective path is clean.

6. **New narrowing result: compilation alone has already reproduced native
   allocator corruption.** In the eleven-process compile-only experiment,
   process 5 aborted after approximately 36 seconds with
   `corrupted double-linked list`, during `.lower(...).compile()` for case
   `3a72be772010e740`. The preceding thirteen signatures had compiled. This
   process used only `jax.ShapeDtypeStruct` input descriptions: no JAX data
   arrays were created and no compiled objective kernel ran. Objective kernel
   execution and transfer/ownership of real input buffers therefore are not
   necessary to trigger this native failure. The exact corruption site inside
   the tracing/lowering/compiler/lifetime path remains unidentified. Final
   count: **two failures among eleven compile-only children**, one SIGABRT and
   one SIGSEGV; the other nine compiled all 56 signatures.

7. **Setting CPU code-generation splitting to one did not fix the crash.**
   The compile-only control with
   `XLA_FLAGS=--xla_cpu_parallel_codegen_split_count=1` lost two of eleven
   children to SIGSEGV during backend compilation. Both stopped while compiling
   signature `df056cdbb8b80b3b` after roughly 197 seconds. The remaining nine
   completed. This flag is rejected as a fix; it does not serialize every
   operation inside the compiler.

8. **Lowering without backend compilation passed the bounded control.** All
   eleven children completed five cycles through all 56 signatures: **3,080
   lowerings**, plus per-case garbage collection and between-cycle cache
   clearing. They did not compile or execute objective kernels. This directs
   the next diagnostic toward backend compilation and its lifetime machinery;
   passing this finite control does not prove lowering is universally safe.

## Portable compiler reproducer

The compiler-only reproducer no longer needs `/tmp` captures or fixture data.
It uses only the captured argument shapes, dtypes and static keyword arguments:

```bash
PYTHONMALLOC=debug .venv/bin/python \
  research/bug-reports/autocal-native-crashes-october-7-2026/evidence/objective-reproducer/replay.py \
  --output /tmp/autocal-compiler-repro --workers 11
```

The output directory must be new. Each child is bounded to 600 seconds by
default. Add `--lower-only --cycles 5` for the lowering control, or `--gdb` for
native backtraces. GDB needs permission to trace child processes; this sandbox
denied `ptrace`, so the approved GDB comparison is running outside it.

The current objective file is byte-for-byte identical to the historical
baseline objective. Its SHA-256 is
`b4a7979205e079ab304522bc5e11fc5088b37fe734e986266caebb8d53803792`.
Each portable child prints the source hash, interpreter/JAX/NumPy versions
and relevant environment settings before compiling.

## Evidence locations

Current working evidence is in the ignored directory:

`output/research/autocal-crash-fix-20261007/`

- `nohook-1.txt`, `nohook-1/summary.json`, `nohook-1/06.txt`: confirmed failure
  without custom native instrumentation.
- `debug-allocator.txt`, `debug-allocator/summary.json`, individual child logs:
  five native failures with Python debug allocation.
- `numpy-control.txt`, `numpy-control/summary.json`, `numpy_control.py`: passing
  JAX-free control.
- `memcheck.txt`, `memcheck-stdout.txt`: timed-out diagnostic.
- `memcheck-affinity.txt`, `memcheck-affinity-stdout.txt`, `memcheck_probe.py`:
  pre-NumPy diagnostic failure.
- `memcheck-outside.txt`, `memcheck-outside-stdout.txt`: approved outside-sandbox
  Valgrind comparison, four complete cases and timeout; no invalid access
  reported in the work reached.
- `nohook-outside.txt`, `nohook-outside/`: approved ordinary outside-sandbox
  comparison, one SIGSEGV and ten complete children.
- `research.md`: objective, budget, hypotheses and next decision.

Replay scripts and the 56 captured inputs currently come from the previous
investigation's `/tmp/autocal-crash-investigation-20261007/`. The original
Python 3.12.3 / NumPy 2.4.3 / SciPy 1.17.1 / JAX 0.9.2 environment is retained.

## Next decision

The eleven-process **compile-only** experiment used
`PYTHONMALLOC=debug`. It supplies `jax.ShapeDtypeStruct` descriptions of the
captured arguments to `.lower(...).compile()`. It creates no JAX input arrays
and never executes a compiled objective. This checks whether kernel execution
or input-buffer lifetime is necessary for the corruption. Scripts and child
logs are in `compile_only.py`, `run_compile_only.py`, `compile-only.txt`, and
`compile-only/` in the current evidence directory. Each child has a 600-second
limit. It completed with two failed and nine successful children.

The lowering-only and single-split controls completed as reported above.
Both used eleven processes and Python's debug allocator. A helper refactor between the original
compile-only run and these controls moves per-case variables into a function;
do not interpret a changed failure rate as caused solely by the compiler flag.

The portable reproducer is now running under GDB in eleven processes outside
the sandbox, with Python's debug allocator and default XLA settings. Inspect
any native backtrace from this compile-only failure before changing production
code. The Valgrind run is finished and did not locate the first invalid access.
Do not adopt dependency upgrades, GC disabling, or worker bounding as a
verified root-cause fix based on the evidence so far.
