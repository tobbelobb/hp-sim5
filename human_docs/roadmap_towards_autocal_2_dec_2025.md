Gcode dynamic input
Torque mode
Encoder readings
Maybe force sensing


We want to develop a script that defines the auto calibration procedure.
I've tried before

The challenges I've faces have been:
 1. The data is not collected automatically by the machine, which makes the procedure labor intensive.
 2. The data I've managed to collect has not been good enough, so I've had to fumble around with excluding the few data points that incur the largest cost, which is labor intensive and not automated.
 3. The found anchor points need to be rotated after the procedure finishes, which can be a very hard and error prone process. I can't ask my users to actually do that themselves, it needs to be automated.
 4. If an anchor or some other parts of physical config for some reason changes, then the whole procedure (data collection, data pruning, optimization, gravity alignment) needs to be re-run.



Challenge 1: Automate Data Collection

Can I do a measurement and recover my starting position?

Can I write a list of gcodes and receive the responses automatically?

What points should I try and move to?
 - Maybe 20 random points within 500 mm pull in each direction,
   then 20 random points within 1000 mm pull in each direction (if anchor estimate is promising)
   then longer if anchors are estimated to still be far away.

There should be one script for each geometrical setup:
 - Slideprinter
 - HP4
 - HP5
 - CubeCorner
 - SkyCam
