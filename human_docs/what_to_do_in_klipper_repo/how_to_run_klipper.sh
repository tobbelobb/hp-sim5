#### Option 1: Make a Linux process that presents itself to Klipper as an mcu.
#  - Put a script in the middle that forwards pty messages between Klipper and the "mcu"
#    while also mirroring the Klipper->mcu messages to the websocket.
#
# First make sure you have the simulated gpio pins on your computer:
./make-fake-pin-chip.sh

# Then launch the "mcu" with

~/repos/hp-sim5/examples/klipper/slideprinter/klipper_linux_mcu_bridge.py --raw-path /tmp/klipper_host_mcu_raw --host-path /tmp/klipper_host_mcu --mcu-bin ~/repos/hp-sim5/examples/klipper/linux_mcu/klipper.elf --dict ~/repos/hp-sim5/examples/klipper/linux_mcu/klipper.dict --klipper-py ~/repos/klipper/klippy/

# Then launch klippy with

~/klippy-env/bin/python ~/repos/klipper/klippy/klippy.py ~/repos/hp-sim5/examples/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg -i ~/repos/hp-sim5/public/examples/gcode/movements.gcode -v -l ~/repos/hp-sim5/attic/klipper.log

#### Option 2: Simulate the whole avr and mirror the incoming messages on to the websocket.
#
# First compile klipper with simulavr support
# $ make menuconfig
# Select
#  AVR atmega644p, enable "low-level configuration options", and enable
# "simulavr software emulation".
# Keep 16 MHz as the cpu and clock speed then do
# $ make
# The created files are required for the avr simulation we're going to run later
# cp out klipper.dict ../hp-sim5/examples/klipper/avr/klipper.dict
# cp out klipper.elf ../hp-sim5/examples/klipper/avr/klipper.elf

# We also need to run simulavr
# Install it like so:
# $ git clone git://git.savannah.nongnu.org/simulavr.git
# $ cd simulavr
# $ export CXXFLAGS="-O3 -march=native -flto -g0"
# $ export CFLAGS="-O3 -march=native -flto -g0"
# $ export LDFLAGS="-flto -Wl,-O3"
# $ make cfgclean
# $ make python
# $ make build
# Then confirm you have the .so file built and in place with
# $ ls ./build/pysimulavr/_pysimulavr.*.so
# Then simulate avr with

PYTHONPATH=~/repos/simulavr/build/pysimulavr ~/repos/hp-sim5/examples/klipper/slideprinter/klipper_avr_bridge.py ~/repos/hp-sim5/examples/klipper/avr/klipper.elf --dict ~/repos/hp-sim5/examples/klipper/avr/klipper.dict --klipper-py ~/repos/klipper/klippy/ --rate 1.0 --keep-noise --parse-debug


# This will continuously hog one cpu core which is not ideal but no show stopper either
# It creates a processes that klipper will think is a real avr microcontroller, which is great for our purposes.
# Now that the mcu is running, start klipper and make it push some movements through.

# First run this in another terminal to make sure we're capturing klippers log output
tail -F ~/repos/hp-sim5/attic/klipper.log
# Then run klipper itself
~/klippy-env/bin/python ~/repos/klipper/klippy/klippy.py ~/repos/hp-sim5/examples/klipper/slideprinter/printer-slideprinter-avr.cfg -i ~/repos/hp-sim5/public/examples/gcode/draw_squares.gcode -v -l ~/repos/hp-sim5/attic/klipper.log
