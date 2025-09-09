
# For batch mode we need a dict. Make an AVR one because we want to simulate extruder which the "Linux process" dict won't allow
# `make menuconfig`
# Choose AVR
# Save and exit
# `make`

~/klippy-env/bin/python ./klippy/klippy.py ~/repos/hp-sim5/examples/klipper/slideprinter/printer-slideprinter-avr.cfg  -i /home/torbjorn/repos/hp-sim5/public/examples/gcode/draw_squares.gcode  -o test.serial -v -d out/klipper.dict
~/klippy-env/bin/python ./klippy/parsedump.py /home/torbjorn/repos/klipper/out/klipper.dict test.serial > test.txt
