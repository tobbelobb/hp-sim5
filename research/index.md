# Research session catalog

One result sentence per session; follow the report for evidence.

- [Latest simulator correction and HP3 autocal failures](sessions/attempt-0b9a865f76514b04930528d43f2c31ea/report.md) — A corrected historical/current pair produced identical encoder sweeps and failed without applying calibration, while a separate open-loop torque check reproduced the simulator correction.

- [Automatic force tuning repairs the HP3 comparison](sessions/attempt-64d28c7e5e6c4e778656afa4639a46c6/report.md) — Current and historical native/wrapper runs both applied the same calibration after replacing the unsuitable manual-force acquisition, with maximum anchor error 4.388 mm.



- [Autocal flag audit and bounded experiment handoff](autocal-flag-audit/report.md) — This audit asks which autocal options deserve further tuning, consolidation or removal.

- [Autocal ellipse flex removal and zero-tension validation](autocal-physical-model-cleanup/report.md) — Lane 05 found that ellipse flex correction never applied on the reference corpus because the collector writes no per-point tensions.
