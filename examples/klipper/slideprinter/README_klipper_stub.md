Klipper → PTY stub → Browser logging
===================================

Goal
  Confirm we can capture raw MCU bytes that Klipper sends to its [mcu] serial
  device and see them in the browser console in the slideprinter demo.

Steps

1) Choose your source

   A) simulavr path (recommended for AVR dev):

   - Start the WS broadcaster with UDP injection (no PTY needed here):

     ```bash
     python -m examples.python.slideprinter.klipper_handler --port 8770 --udp-in-port 8771
     ```

   - Start the AVR simulator and mirror TX bytes to the UDP port:

     ```bash
     # In your Klipper repo (after building out/klipper.elf for atmega644p + simulavr)
     python -m examples.klipper.slideprinter.avrsim_bridge \
       --port /tmp/pseudoserial --baud 250000 \
       --udp-host 127.0.0.1 --udp-port 8771 \
       out/klipper.elf
     ```

   B) PTY-only path (no simulavr):

   - Create a PTY and WS bridge that Klipper can open directly:

     ```bash
     python -m examples.python.slideprinter.klipper_handler --port 8770 --symlink /tmp/pseudoserial
     ```

2) Configure Klipper to use `/tmp/pseudoserial`

   In your Klipper printer cfg (the repo has an example at
   `examples/klipper/slideprinter/printer-slideprinter-avr.cfg`), make sure:

   ```ini
   [mcu]
   serial: /tmp/pseudoserial
   ```

3) Start Klipper host and feed it G-code

   - Recommended: run Klipper host in the normal mode so it opens `/tmp/printer`
     (or `~/printer_data/comms/klippy.serial` on newer setups) and connect
     OctoPrint to it. Send a G-code file so Klipper emits MCU traffic.

   - Alternative: try batch mode if your version still opens the MCU serial while
     reading a `-i` G-code file. This is less reliable across versions.

     ```bash
     # Example only; adjust paths for your setup
     ~/klippy-env/bin/python ./klippy/klippy.py \
         /path/to/printer-slideprinter-avr.cfg \
         -i /path/to/draw_squares.gcode -v
     ```

4) Open the JS slideprinter demo and enable logging

   - Run `npx vite` for the site, and start the Python slideprinter server if needed.
   - Open the slideprinter demo in your browser (as you normally do).
   - In the top bar, check "Klipper raw in" and leave port as 8770 (or your choice).
   - You should see `KlipperHandler: … bytes: …` lines in the browser console when
     Klipper is active and sending MCU bytes.

Next steps
  Once raw bytes flow, we can add translation in JS or Python to convert the
  MCU protocol into the JSON commands consumed by the RemoteSpoolSystem.
