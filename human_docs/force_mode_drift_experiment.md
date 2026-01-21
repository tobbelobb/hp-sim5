I'm mapping the hp-sim hysteresis effects of running series of force mode commands after one another.
A force mode command can looks for example like this: `M569.4 P40.0:41.0:42.0 T1:2:3`
Here, "40.0:41.0:42.0" is the colon separated list can addresses of the three motors, and "1:2:3" is the colon separated list of
how many Newtons of force pull we should apply with each of those motors.
The firmware responds to a force mode command by telling us how much torque it applies to each motor, in order, like this: `-0.030000 Nm, -0.060000 Nm, -0.090000 Nm,`

After applying a force mode command we typically wait until the encoders are stable and make an encoder reading, for example like this:
`M569.3 P40.0:41.0:42.0`
Again, the P-parameter list "40.0:41.0:42.0" denotes the can addresses of the motors we want to read the encoders of.
The firmware responds to an encoder reading command by telling us how many degrees each motor has rotated away from it's reference point, like this: `[-505.66, 433.51, 131.04, ]`

Before we start an experiment, or a series of commands, we zero everything, which means the end effector gets moved to the Cartesian origin point,
and the encoders get their reference point set so that `M569.3 P40.0:41.0:42.0` returns exactly zero, that is [0, 0, 0, ].

I also watch the simulated Slideprinter, so I can sometimes infer "No movement" or similar visually.
In those cases I will write my observation right into the log manually, for example like this: " -> No movement".

I get this results:

Series 1:
<zero_everything />
M569.4 P40.0:41.0:42.0 T1:1:1 -> No movement

Full log:
> M569.4 P40.0:41.0:42.0 T1:1:1
-0.030000 Nm, -0.030000 Nm, -0.030000 Nm,
<wait_until_encoders_stable />
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[-0.42, 0.53, -0.42, ]

That's well and fine. The anchors are symmetrically placed around the origin so we expect that pulling equally hard in all three lines
results in no movement at all since we're in a static equilibrium.

Series 2:
<zero_everything />
M569.4 P40.0:41.0:42.0 T2:2:2 -> No movement

Same result, no surprises.

Series 3:
<zero_everything />
gcode> M569.4 P40.0:41.0:42.0 T2:1:1
> M569.4 P40.0:41.0:42.0 T2:1:1
-0.060000 Nm, -0.030000 Nm, -0.030000 Nm,
<wait_until_encoders_stable />
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[-505.66, 433.51, 131.04, ]
-> Here we surprisingly get much more movement in motor 41.0 than in motor 42.0.
-> The trace I see isn't even straight, it's a curve.
-> Motor 41.0 has yielded more line than motor 42.0, although they were set to the same force and torque setting.
-> I was expecting a move directly towards motor 40.0, with equal amounts of line wound off of spools 41.0 and 42.0,
-> but I'm observing that motor 41.0 lets out more line than motor 42.0
gcode> M569.4 P40.0:41.0:42.0 T1:1:1
> M569.4 P40.0:41.0:42.0 T1:1:1
-0.030000 Nm, -0.030000 Nm, -0.030000 Nm,
<wait_until_encoders_stable />
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[-336.86, 308.52, 56.36, ]
gcode> M569.4 P40.0:41.0:42.0 T2:2:2
> M569.4 P40.0:41.0:42.0 T2:2:2
-0.060000 Nm, -0.060000 Nm, -0.060000 Nm,
<wait_until_encoders_stable />
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[-214.59, 229.10, -1.24, ]

My base expectation was that we would slide linearly back towards zero on all axes when we equalized the force in every direction.
Instead, we have end up far away from the path between [0, 0, 0, ] and [-505.66, 433.51, 131.04, ].
It looks like motor 42.0's dominance has continued to increase, since its value has reached, and even overshot, it's zero point while neither
of motors 40.0 or 41.0 are back to their starting position.
There's some hysteresis in the system, and all of it has been taken up by motors 40.0 and 41.0.

Series 4:
<zero_everything />
gcode> M569.4 P40.0:41.0:42.0 T1:2:1
> M569.4 P40.0:41.0:42.0 T1:2:1
-0.030000 Nm, -0.060000 Nm, -0.030000 Nm,
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[239.09, -440.78, 239.09, ]
gcode> M569.4 P40.0:41.0:42.0 T1:1:1
> M569.4 P40.0:41.0:42.0 T1:1:1
-0.030000 Nm, -0.030000 Nm, -0.030000 Nm,
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[92.55, -178.89, 92.55, ]
gcode> M569.4 P40.0:41.0:42.0 T2:2:2
> M569.4 P40.0:41.0:42.0 T2:2:2
-0.060000 Nm, -0.060000 Nm, -0.060000 Nm,
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[13.54, -27.36, 13.54, ]

In this series we get what I expected in the first series.
First a clean symmetrical drift towards the strongest motor, in this case motor 41.0: `[239.09, -440.78, 239.09, ]`
Then a straight drift back, in two steps: `[92.55, -178.89, 92.55, ]` and `[13.54, -27.36, 13.54, ]`
with some hysteresis remaining, so we don't get all the way to zero, but we moved almost linearly towards zero until we stopped.
(The axes are not perpendicular, so we don't expect a linear relationship between the three encoders in general, but on small distances we can expect at least a roughly linear relationship.)

Series 5:
gcode> M569.4 P40.0:41.0:42.0 T1:1:2
> M569.4 P40.0:41.0:42.0 T1:1:2
-0.030000 Nm, -0.030000 Nm, -0.060000 Nm,
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[131.04, 433.51, -505.66, ]
gcode> M569.4 P40.0:41.0:42.0 T1:1:1
> M569.4 P40.0:41.0:42.0 T1:1:1
-0.030000 Nm, -0.030000 Nm, -0.030000 Nm,
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[56.36, 308.52, -336.86, ]
gcode> M569.4 P40.0:41.0:42.0 T2:2:2
> M569.4 P40.0:41.0:42.0 T2:2:2
-0.060000 Nm, -0.060000 Nm, -0.060000 Nm,
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[-1.24, 229.10, -214.59, ]

This series more similar to the surpirsing Series 3 again.
First off, we don't get a linear, symmetrical drift towards the strongest anchor (in this case motor 42.0).
We see that motor 41.0 has yielded much more line than motor 40.0 although they were commanded to the same force and torque setting.
The trace is once again is a curve, not a straight line, just like in Series 3.

I want you to look at possible reasons why motor 41.0 (also known as "Spool B" or "Motor B" in some contexts) acts weaker than the other two motors.
You can look for reasons in ./examples/usd_scenes/slideprinter_multi_unit.usda, which defines the simulated scene.
There could be something about distances, angles, materials, radii or something.
You can also look in hp-sim/assets/hp-sim.js which defines the simulator code.
There could be something about the order in which constraints are applied or something.

-----------------

I actually found that the reason we don't get straight lines is that the motor coggign effects
takes away the single equilibrium position that smooth torques and statistical mechanics would predict.
Instead, we get multiple zones of equilibrium spread out near the origin point, not neccesarily even and not neccesarily with
an equilibrium particularly close to the origin.
We will not get an "average sample point" very close to the origin through pure torque mode drift, even if we make infinitely many samples.
That's an interesting result
