# First compile klipper with simulavr support
# make menuconfig
# Select
#  AVR atmega644p, enable "low-level configuration options", and enable
# "simulavr software emulation".


~/klippy-env/bin/python ~/repos/klipper/klippy/klippy.py ~/repos/hp-sim5/examples/klipper/slideprinter/printer-slideprinter-avr.cfg -i ~/repos/hp-sim5/public/examples/gcode/draw_squares.gcode -l ~/repos/hp-sim5/attic/klipper.log -v

# Then have the logs rolling in another terminal with
tail -f ~/repos/hp-sim5/attic/klipper.log


# We also need to run simulavr
# Install it like so:
# $ git clone git://git.savannah.nongnu.org/simulavr.git
# $ cd simulavr
# $ make python
# $ make build
# Then confirm you have the .so file built and in place with
# $ ls ./build/pysimulavr/_pysimulavr.*.so
# Then simulate avr with

PYTHONPATH=/home/torbjorn/repos/simulavr/build/pysimulavr /home/torbjorn/repos/hp-sim5/examples/klipper/slideprinter/klipper_avr_bridge.py /home/torbjorn/repos/hp-sim5/examples/klipper/avr/klipper.elf --dict /home/torbjorn/repos/hp-sim5/examples/klipper/avr/klipper.dict --klipper-py /home/torbjorn/repos/klipper/klippy/

# This will continuously hog one cpu core which is not ideal but no show stopper either
