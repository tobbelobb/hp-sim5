Feature Request: Http endpoint for RrfCommander.

In the hp-sim app I've implemented a class called RrfCommander (see the file examples/js/slideprinter/rrfCommander.js). It simulates the real ReprapFirmware by reading pre-computed movement commands, stored in .csv or .can files, and sending them to the physics engine with the right timing.

The program who pre-computed the movements and stored them in .csv format is called `rrf_simulator`. It is compiled and invoked like this:
```
cmake --build RRF/build --target rrf_simulator -j
./RRF/build/rrf_simulator --vsd RRF/run/vsd --gcode gcodes/draw_squares.gcode --can-log logs/draw_squares.csv -c sys/config_slideprinter.g
```

All the code related to ReprapFirmware is in the RRF directory. The ReprapFirmware docs are in RRF/RepRapFirmware.wiki/.

Now we want RrfCommander or a helper class to do something more dynamic. We want it to replicate the behavior of real ReprapFirmware's http endpoint.
I want to be able to send `M569.4 P40.0 T0.001` to the gcode endpoint (http://duet3.local/machine/code/ or any other simulated endpoint) like this:
```
$ curl --silent http://<some_url>/machine/code/ -d \"M569.4 P40.0 T0.001\" -H \"Content-Type: text/plain\"
```

... and have the StepperMotorSystem in the actual physics simulation enable torque mode at 0.001 Nm for the motor with address 40.0, and also to get the response \"0.001000 Nm,\".
(Sending a 0 torque sets the motor back in position mode. The expected behavior of M569.4 is documented in ai_docs/M569.4).
This behavior should come from invoking `rrf_simulator` which has the logic compiled in. The M569.4 logic I want can be studied in RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp.

An example of how the http endpoint will be used can be seen in ai_docs/get_auto_calibration_data_automatically.sh and ai_docs/use_params.sh.

I believe RrfCommander should interact with the RepRapFirmware Object Model, but I'm not sure about the details how and why. This is what the introduction to the RepRapFirmware Object Model says in the wiki:
```
All Duet software projects (RepRapFirmware, Duet Software Framework) provide a central 'Object Model' (OM) that replicates the entire machine state, storing configuration and sensor data. This data model is synchronized with Duet Web Control as well as stored in its backend, and can be accessed by Gcode and meta Gcode commands, macros and external data requests. This allows for powerful control and feedback systems to be built.
```

I believe the current state of Hangprinter's Object Model is a bit lacking. Investigate RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp to find out.

The doc about RRF HTTP requests is here: RRF/RepRapFirmware.wiki/HTTP-requests.md
It's really the gcode endpoint that's the most important. Whatever's sent there should be sent to rrf_simulator which should generate a .csv file or stream or just return value that contains enough info for RrfCommander to \"do the right thing\". We want to support as many gcodes as possible, but start with G1 and M569.4.

A long auto-generated overview of the Object Model is here: RRF/RepRapFirmware.wiki/Object-Model-Documentation.md
More info about the Object Model can be learned by studying RRF/DuetSoftwareFramework/src/DuetAPI/ObjectModel/ObjectModel.cs and neghboring files.
