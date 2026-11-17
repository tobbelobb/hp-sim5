# Wrong results in the firmware and motion-command layer

Four reproducible wrong results: RRF's host build is off by one microstep at standstill (bounded, not cumulative: a repeated failing unit stays at X = 0.9875 mm for n = 1 and n = 100 waits), multi-driver CAN boards are unsupported end to end (CSV, `.can`, converter and axis map all collapse a board to one axis, turning 15 mm of travel into 8.3 mm of phantom extrusion), the default Klipper MCU player drops the first queue_step batch of every stepper but the first (0.041 mm effector offset on the shipped slideprinter, constant for the whole stream; reached by uploading a `.txt`/`.serial` dump, not by the preset buttons), and the raw Klipper pipeline applies 478 extruder steps to spool axis D; velocity/acceleration limits, step quantization and queue handling otherwise match the closed form.

[Full report and evidence](report.md)
