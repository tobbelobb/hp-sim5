


/read examples/python/slideprinter/slideprinter_common.py
/add examples/js/slideprinter/slideprinter_common.js examples/js/slideprinter/setupScene.js

I have two implementations of virtually the same program, Slideprinter, a simulation of a cable driven robot rendered in the browser. The python implementation has many features that the js implementation is currently lacking:

 - Spools are stepper motors applying force, not just freely rotating.
   See StepperMotorSystem and StepperMotorComponent in @examples/python/slideprinter/slideprinter_common.py

Task: Implement a StepperMotorSystem and StepperMotorSystem in @examples/js/slideprinter/slideprinter_common.js
Task: Integrate them in the appropriate places in the js slideprinter program (eg @examples/js/slideprinter/setupScene.js). Never mind the RemoteSpoolSystem and how it
uses StepperMotorComponent and SpoolStateComponent for now. We will implement the js equivalent of RemoteSpoolSystem in a later task.

I have two implementations of virtually the same program, Slideprinter, a simulation of a cable driven robot rendered in the browser. The python implementation has many features that the js implementation is currently lacking:
 - It has a RemoteSpoolSystem in @examples/python/slideprinter/server.py which reads commands from a websocket and translates them into controlled movements of the stepper motors.
Task: Implement a RemoteSpoolSystem in the js slideprinter program that reads from a websocket like the Python version does. Use StepperMotorComponent and SpoolStateComponent. Make sure to use camelCase in js code even if the Python code uses snake_case.

 - The Python RemoteSpoolSystem is fed by a MoveCommander that's defined in @examples/python/slideprinter/move_commander.py and runs in its own process.
   The MoveCommander is able to read .gcode files from disk and translate them into (linear) move commands that gets sent to the simulation server in a timely manner.
   The MoveCommander simulates an external 3d printer firmware, and will be replaced with other commanders based on real Klipper and ReprapFirmware code in later stages of this project.
   Knowing which move commands to apply to which spools/motors is done by mapping axis names to entity ids. The axis names are stored inside SpoolStateComponent.

Task: Create a js equivalent of the MoveCommander. In the js version it can be it's own thread or something, it doesn't need to be its own separate process, as long as it's executing without interfering with the main update-loop of the simulation. It should read its file from an upload button on the index.html front page, not expect a file name as an argument.

 - The ExtruderSystem and ExtruderComponent defined in @examples/python/slideprinter/slideprinter_common.py
   ExtruderComponent is already defined in @examples/js/slideprinter/slideprinter_common.js
   It is already used to render extrusions in @examples/js/flipper/renderSystem.js
   However, the python ExtruderSystem has no js equivalent yet.
