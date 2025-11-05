We need a slideprinter config file, similar to Klippers slideprinter.cfg.
 - 3 anchors
 - 1 extruder connected via CAN

ReprapFirmware needs to actually support a 3 anchor machine.

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
