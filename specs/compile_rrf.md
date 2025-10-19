# Background

This repo contains a hp-sim app that simulates a 3d printer.
The simulated 3d printer is controlled based on batch output from the Klipper 3d printer firmware, that's found in the klipper submodule.

The klipper batch mode reads a .gcode file as its input and outputs a .serial file containing commands that would usually have been sent synchronously to a mcu.
Most notably, the .serial file contains `queue_step` commands that informs the mcu (and our simulation) exactly when to step each motor.

An example of a .serial file is available at public/examples/mcu_commands/draw_squares.serial.
It's a binary format, but a human readable parsed version can be found at public/examples/mcu_commands/draw_squares.txt.
Documentaton that explains how these commands should be interpreted by the mcu and our simulation, is found at ai_docs/Klipper_MCU_Commands.md

The batch mode, and how to access it (compile configuration and so on) is described in ai_docs/Klipper_Debugging_tools.md.

Our simulation app reads the Klipper output (either .serial or .txt) via the KlipperCommander class in examples/js/slideprinter/klipperCommander.js.


# Feature Request

Our simulator should also support another 3d printer firmware, called ReprapFirmware.
This means we need a similar batch mode in ReprapFirmware as we have in Klipper.
However, ReprapFirmware currently doesn't have such debugging tools.
It doesn't even compile to a x64 binary.


# Step to Focus on

Find out what core part of ReprapFirmware we need to preserve in order to be able to generate something like a sequence of `queue_step` commands, but with ReprapFirmware.
Find out what interfaces we can and should stub (Wifi, SPI, WebControl, etc).
I'm particularly interested in the CAN interface.

Two possible ways to get the batch dump of step signals and timings could be:

 1. Capture CAN packets.
 2. Create our own protocol and plug it into the x64 version of ReprapFirmware following the pattern that the CAN protocol and interface uses in the codebase.

I think option 1 is the most promising.
We should try to define all of ReprapFirmware's motors as having CAN connected
external drivers, and to then generate/capture stepping and synchronization information based on the CAN packets.
The CAN protocol is described in RRF/CANlib/doc/Duet3CAN-FDProtocol.md
To understand more about how it works, I recommend checking out at least RRF/CANlib/src/CanMessageFormats.h, RRF/CANlib/src/Duet3Common.h, RRF/CANlib/src/CanId.h, and RRF/CANlib/src/RRF3Common.h.

However, the details of the CAN protocol and so on are only important after we've managed to compile a ReprapFirmware binary for our x64 host machine.
To learn how RRF is normally built, and which targets it's normally built for, look into RRF/RepRapFirmware.wiki/Home.md and RRF/RepRapFirmware.wiki/Building-RepRapFirmware.md.
To learn about how Eclipse usually builds ReprapFirmware for its normal microcontroller targets, see .cproject.

We don't want to build via Eclipse. We want a normal Makefile.
We probably need to build a Hardware Abstraction Layer, a "fake HAL" in CoreN2G for our purposes as well.
ReprapFirmware is highly dependent on the interrupt‑driven microcontroller environment and the supporting hardware libraries, so it needs more than just a new Makefile.
Building RRF for a desktop machine requires writing a complete hardware‑emulation layer to replace the MCU‑specific functions (timers, stepper‑driver SPI/I²C control, ADCs, etc.),
and possibly porting FreeRTOS or substituting an equivalent scheduler for the host OS.

All that sounds very complicated, but it might not be, because we don't need true realtime performance.
We just need to compute the correct timings and to dump everything into a file, which will be read later,
so the process of generating the file is actually not timing sensitive.
To emulate a clock we can simply increment a variable or something,
interrupts can be simple callbacks, and so on.

I have added RRF/DuetSoftwareFramework and RRF/DuetWebControl to my repo because they are normally used to feed ReprapFirmware the gcode.
They are compilable to x64 already I believe, so we should use it for part of our pipeline.
Just use the websocket interface, not the SPI, or REST API ones.


# Deliverables

I want an incremental plan that tells me in detail how to build a Makefile that can compile
 - one file, then
 - one more, then
 - ..., then
 - test sending a short gcode to our x64 ReprapFirmware binary, and have it echo the text, then
 - Include the config handling, so we can configure our "printer" with a config.g file
 - then gradually work our way through to getting the "batch mode" with .serial/duetCAN protocol output.

A detailed plan like that based on all the available documentation.
