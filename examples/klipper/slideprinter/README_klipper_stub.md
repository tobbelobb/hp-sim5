Klipper → PTY → WebSocket → Browser logging
==========================================

Goal
- Capture raw MCU bytes that Klipper sends to its [mcu] serial device and log
  them in the browser console in the slideprinter demo.

Recommended: single merged bridge
1) Start the AVR simulator + PTY + WebSocket bridge (no UDP hop):

   ```bash
   PYTHONPATH=/path/to/simulavr/build/pysimulavr \
     /path/to/hp-sim5/examples/klipper/slideprinter/klipper_avr_bridge.py \
     /path/to/klipper.elf
   ```

   - Defaults: PTY at `/tmp/pseudoserial`, WS at `localhost:8770`.
   - Options: `--machine`, `--speed`, `--rate`, `--baud`, `--port`,
     `--trace`, `--tracefile`, `--ws-host`, `--ws-port`.

Alternative: PTY-only bridge (no simulavr)
- If you want to test with a raw PTY and no AVR sim, you can still use:

  ```bash
  python -m examples.python.slideprinter.klipper_handler --port 8770 --symlink /tmp/pseudoserial
  ```

Configure Klipper to use `/tmp/pseudoserial`
- In your Klipper printer cfg (see `examples/klipper/slideprinter/printer-slideprinter-avr.cfg`):

  ```ini
  [mcu]
  serial: /tmp/pseudoserial
  ```

Start Klipper and feed it G-code
- Recommended: run Klipper normally so it opens `/tmp/printer` (or
  `~/printer_data/comms/klippy.serial` on newer setups) and connect OctoPrint.
  Send G-code so Klipper emits MCU traffic.

Open the slideprinter demo and enable logging
- Run `npx vite` for the site (and Python slideprinter server if needed).
- Open the slideprinter demo in your browser.
- In the top bar:
  - Check "Klipper raw in" (this now connects immediately on toggle).
  - Leave port as `8770` (or your WS port).
- You should see `KlipperHandler: … bytes: …` lines in the browser console when
  Klipper is active and sending MCU bytes.

Next steps
- Once raw bytes flow, we can add translation in JS or Python to convert the MCU
  protocol into the JSON commands consumed by the RemoteSpoolSystem.
