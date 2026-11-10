# Autocal crash investigation — current findings

**Follow-up:** see [autocal-crash-followup.md](autocal-crash-followup.md) for the
new investigation. Compilation with shape/dtype descriptions alone reproduced
native failures; executing objective kernels is not required. A portable
reproducer and the argument signatures are now saved beside the report.

You can debug this ordinary numerical software here. The cybersecurity banner appears to be a false positive; I cannot inspect or override it. These notes are saved locally so you can read them even if chat output is hidden.

**No verified fix yet. Do not upgrade JAX as a claimed fix.**

- I searched upstream JAX issues and release notes. I found no verified defect and fixed version matching your CPU, float64, JAX 0.9.2 crash report. Similar reports have different triggers or were fixed before 0.9.2.
- I tested JAX/jaxlib **0.11.1 in an isolated directory**, retaining your existing environment. Full calibration runs still crashed, and some completed calibrations differed materially from the reference results.
- New native traces show faults in NumPy `PyArray_SafeCast`, reached by `np.linalg.lstsq(...).astype(...)`. GDB captured an invalid output pointer (`0x1a`). This identifies a crash site, not the earlier cause of corruption.
- A smaller **JAX 0.11.1 objective replay also segfaulted during garbage collection**, without SciPy optimization or the calibration control loop. This gives us a reduced reproducer, but does not prove it has the same cause as your original 0.9.2 failures.
- Three additional full 11-worker runs on the original JAX 0.9.2 completed successfully. That does not invalidate the original failures or prove a fix. Two workers and disabling GC remain unproven workarounds.

**New result:** repeated objective-only execution also reproduced a garbage-collection segfault on the original **JAX 0.9.2**: one of 11 processes failed. The other 10 each completed 5,600 evaluations. This substantially narrows the problem to the objective/JAX/native lifetime path; the SciPy optimizer and the calibration control loop are not required to trigger it.

**Current next step:** confirm this reduced failure without the native trace hook and run allocation diagnostics on it. The first invalid write/free and exact faulty dependency remain unidentified; the version upgrade is rejected as a fix.

Your repository and `.venv` have not been changed. Experiments use a historical source snapshot and fresh fixture copies under `/tmp/autocal-crash-investigation-20261007`. Evidence and fuller notes will be preserved beside this file as the diagnostics finish.

Sources: [JAX release notes](https://docs.jax.dev/en/latest/changelog.html), [older weak-reference crash, fixed before 0.9.2](https://github.com/jax-ml/jax/issues/30517), [similar CUDA-only compilation corruption report](https://github.com/jax-ml/jax/issues/40142), [OpenAI documentation acknowledging unrelated safeguard triggers](https://learn.chatgpt.com/docs/cyber-safety).
