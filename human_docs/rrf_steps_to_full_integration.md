We need a slideprinter config file, similar to Klippers slideprinter.cfg.
 - 3 anchors
 - 1 extruder connected via CAN

ReprapFirmware needs to actually support a 3 anchor machine.

 Tasks:
 1. Confirm we're actually capturing movements for all four CAN "boards"
 2. Confirm the "print time" printed to console is accurate
 3. Run tests with Hangprinter_logo6.gcode.
   - How large time step can we choose, and still get deterministic output?
   - Why is that?
   - Can we generate this time step dynamically instead of just using a fixed size time step?


We need a KlipperCommander equivalent.
Or we need to translate the can.jsonl into klipper mcu commands
  They're different frequencies, I don't know how that affects things.
  Klipper 10 MHz
  RRF 48 MHz, 48/64 = .750 MHz steprate

The RrfCommander needs to be integrated into hp-sim.

We need to compact the jsonl files. We can't serve 200 MiB files.

We need a drop-down to select between draw_squares.jsonl and Hangprinter_logo6.jsonl.

ReprapFirmware also needs to features to match Klipper:
 - Ability to turn off spool buildup
 - The new flex compensation
