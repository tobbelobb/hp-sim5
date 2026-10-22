# Headless JavaScript physics profiling

Status: complete.

## Abstract

The production headless JavaScript HP3/HP4 physics was measured on fixed firmware logo trajectories and complete recorded autocal movement/settling schedules, with layering enabled and disabled. Eight cases used three uninstrumented repetitions and separate system-timing, native V8 CPU and allocation-sampling runs: 48 measured trajectories and 7,544,232 physics updates. Every profiler mode matched its baseline numerical checkpoints exactly. The constraint solver, attachment updates and over-correction dominate runtime. Allocation is substantial, but reported GC contributes about 1–2% of replay time. Layering removal speeds the logo workload but gives little benefit on the autocal schedules and can change the physical trajectory substantially. The faithful HP3 autocal replay also reproduces 1,956 saved encoder reads to floating-point rounding. These results establish a reproducible baseline; no physics optimization was made. Measurements exclude firmware, transport, rendering and recording costs, and frozen settling schedules cannot establish adaptive collector or hardware performance.

## Topics

HP3, HP4, JavaScript, headless, performance, profiling, V8, allocation, garbage collection, autocal settling, layering.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Attachment is always the largest system | P01: eight system profiles | Rejected | Solver leads seven of eight cases |
| Temporary allocation is substantial | P02: eight native allocation profiles | Supported | Hundreds of KiB to more than 1 MiB allocated/update |
| GC is the dominant current bottleneck | P02: GC events and CPU profiles | Rejected | Reported GC approximately 1–2%; solver/attachment math dominates |
| Disabling layering consistently speeds this simulation | P03: all on/off pairs | Mixed | Logo improves; autocal changes little, with physical divergence |
| The profiling modes preserve numerical checkpoints | P04: 48 measured trajectories | Supported | Exact checkpoint hashes within every case |

## References

- [Full report](report.md): results and scope.
- [Research notebook](research.md): constraints and experiment choices.
- [Matrix](matrix.json): measured numerical/timing evidence.
