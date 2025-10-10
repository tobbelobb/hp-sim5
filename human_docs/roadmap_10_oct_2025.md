# Reliability of hp-sim5

We have pinned down that the RemoteStepperSystem waits (blocking) for move commands from the KlipperCommander reading .serial files.
We need to make sure KlipperCommander sends empty Move commands to further the simulation in case no steps were planned for a given timespan.

We want a guarantee that there are no buffering or similar bugs in klippers pacer or commander.


# General Strategy

We need several firmware features in Klipper.
We need the same firmware features that are already in ReprapFirmware.
We need ReprapFirmware to plug into hp-sim5.

 - Buildup Compensation
 - Flex compensation
 - Forward transforms
 (- Torque mode)
 (- Encoder readings for auto calibration)
 (- hp-mark)
 (- Force sensing)

Once a new feature in Klipper, or preferrably several of them, they are tuneable as parameters in the .cfg file.

The general layout of the machine needs, thinks like

 - The number and approximate position of anchors, and
 - what sensors are available

... need to be statically set by a human, but we can optimize programmatically
over anything deemed a mechanical- or software-feature (see later in the text) like:

 - Mover size (mechanical)
 - Gearing (mechanical)
 - Buildup Factor (software)
 - Pretension (mechanical)
 - Flex compensation (software)


# ReprapFirmware Support in hp-sim5

We really want to use ReprapFirmware in batch mode on the x64 architecture, on the desktop.
That way ReprapFirmware and Klipper get hot-swap interchangeable at the .serial layer.

Anything else will make it too hard to

 - Simulate real-time behavior of the stepper/step signals. We need signals with already-computed perfect time stamps.
 - Maintain in the deeper abstraction layers. Like the simulated Kalman Filter prototype or stepper timing inside
   the physics engine. We can't have separate implementations for for signals coming from Klipper and ReprapFirmware.

From my understanding, the following steps towards ReprapFirmware support in hp-sim5 are feasible:

 - Compile ReprapFirware or x64.
 - Define all steppers as CAN-connected.
 - Find or implement a batch mode in ReprapFirmware that doesn't wait for anything, just computes the steps.
 - Capture the CAN-commands in a binary .can file or something.
 - Translate
   1. .can -> .serial, or
   2. .serial -> .can, or
   3. .serial -> .hpsim and .can -> .hpsim
   ... where .hpsim is some format that we define ourselves to work well with hp-sim5.

I have not seen a hard requirement to define our own protocol yet.
There might be reasons why we want a self-designed unified interface later, but try to postpone or avoid that.
Go for alternative 1 first.


# Kalman Filter

The Kalman Filter approach needs some sensor input, so establishing that is prior to developing the Kalman Filter feature.

For a live Kalman Filter self-calibration approach we need to run a live firmware during print?
At least we can't pre-compute every step/dir signal, we need the ability to:
 - Run Kalman cfg recompute step
 - Recompute remaining part of the print based on that

This is probably possible in pure sim by keeping track of which gcode we're currently printing, and then
at the end of say every 10th gcode command,
or aften a given number of millimeters of travel away from the starting point,
we recompute the .cfg file.
We then truncate the gcode file, keeping only the lines we haven't already printed.
The new.cfg + truncated.gcode are processed by Klipper/ReprapFirmwareBatcher to generate remaining.serial.
In KlipperHandler or similar class we throw away our old .serial file and hot-swap in our new remaining.serial.

If we are cool, we could also use the timesteps that Klipper generate to help us hot-swap seamlessly from
old.serial to remaining.serial, and not use the gcode lines or measures like that at all.
The `queue_step` command and .serial protocol in general works with relative positions at all times,
so this should make no difference compared to how a live Klipper would work.

The above procedure would require full stop (wall clock time) upon every recompute in the simulation.
However, in simulation time, and this in a real life scenario, the jump from old.serial to remaining.serial
will be seamless.

In practice I'm not sure we would want the Kalman filter to be on during print, because the forces arising
from the print head partially colliding with the print creates so unpredictable forces.
Or could we make this work reliably even during print?
At first we shouldn't bother with extrusions at least.



# Flex Compensation

This feature is already implemented in ReprapFirmware and it's easy to get into Klipper.

It should be developed right away because it's best tested in the simplified environment that we already have:

 - 2D
 - No buildup compensation (isolate flex compensation from other problems)


## Pretension Testing

There's currently no way in hp-sim5 to manually tighten or loosen lines in the UI,
in a way that affects a print's quality score.
There's a full reset before any print.
This is a good thing, because then the test is more contained inside the combination of

 - .cfg/config.gcode,
 - .gcode file
 - .usda file,

... Making the test repeatable and batchable.

Pretension can be created either with a custom line in the gcode file, or better: with a custom .usda file.

hp-sim5 is currently geared towards handling different .usda files so we could just define
a series of pretensioned designs in order to test different pretensions.
This makes sense.

The amount of pre-tension is part of the mechanical design, not part of the gcode.
Move an anchor -> change the optimal pre-tension. Anchor location is in .usda.
Change line stiffness -> change the optimal pre-tension. Line stiffness is in the .usda.
The pretension mm should be stated explicitly in the .usda, maybe at the CablePath level,
so the pretension can be distributed evenly among the CableJoints.

Feature wish:
The `restLength` should not have to be stated exlicitly in the CableJoint prim. Just the start- and end-point.

We will maybe not want to test lots of pre-tension values or optimize over them so much,
since the optimal pre-tension is already computed by the flex compensation algorithm.
Hopefully we may just confirm that the computed optimal is indeed showing optimal simulated quality outcomes
compared to all other pre-tensions, that there's at least a local optimum around that level at all times,
and infer that this is probably a global optimum.


# Buildup Compensation

This feature, in a very basic form, is easy enough to implement into Klipper, and is already inside ReprapFirmware.
But we  shouldn't be happy with the very basic form.
It assumes:

 1. a uniform, gradual buildup
 2. no dependence on line tightnes,

.. none of which are true.

We want to properly handle real spool buildup.
To properly verify that we do, we need to simulate spool buildup really well.
The need for such a simulation was a driving reason to create hp-sim5 in the first place.

The proper and useful way to simulate buildup is in 3D, with a long chain of rigid bodies,
wrapping around whatever mechanics the spool might have.
There's a border, or a "gate", where our simple non-building CableJoints translate into a long chain of capsules.

Here, the new Nvidia approach to collision detection makes a lot of sense.
We should integrate that Nvidia paper about force fields around every particle.
Our earlier experiments with Nvidia IsaacSim rigid bodies showed that the capsules tended to float right
through the walls of our spool cylinder, particularly during high forces and rapid movements.
Fine tuning all the parameters to minimize wall penetration was difficult, time consuming, frustrating, and error prone.

A good approach would be to implement this kind of "gate" right into our 2D CableJoints library
and to test it thoroughly in 2D before we step up to 3D.

The next development step for "gate" cable joints could either be to go fully 3D or to develop
a hybrid 2D/3D approach.

It's reasonable to make the "gate" be a 3D plane, and we could even transition between 2D and 3D simulation at this
gate step, allowing a simulated 2D Slideprinter to have 3D simulated spools, losing only a mimimal amount
of Z-direction information at the gate, which is compensated by pure length differences in XY space cable length.

It might be a cleaner and work saving approach to just go full on 3D Cable Joints though, since we want 3D for other
stuff anyways, and just pay the performance price.
I honestly don't know which approach is best.
The pure 3D one has the fewest unknown unknowns I guess.


## Buildup Factor Testing

This is a parameter that will be specified in the .cfg file, and maybe updated by Kalman Filter.
There needs to be a quick way to go from gcode to test result.

New.cfg -> new Klipper processing -> new.serial -> Run test and read quality result.

Right now we need to
 - push the gcode through Klipper manually, and then
 - upload new.serial, and
 - click "Print Square"
 - click "Finish ASAP"
 - Note down result and compare to previous runs

... after every change to the cfg file or to Klipper itself.


## Single Experiment Mode

To shorten the test loop for Klipper and cfg changes we could

 - Upload .cfg files directly in the hp-sim5 interface, which triggers:
   * Klipper to process draw-squares.gcode + new-setting-1.cfg into new-setting-1.serial.
     - The basename of the .cfg (new-setting-1 in this example) becomes the experiment name.
     - The experiment name should describe the cfg diff compared to a defined and tested baseline .cfg.
   * hp-sim5 to start a new print, and
   * Collect final quality score in the Quality Scores List, including new-setting-1.serial name.

We should probably include klipper right in the hp-sim5 codebase, since we reference it already in our scripts.

This is a precursor to, and should be designed to be easy to buid out to a more fully featured Test Batch Mode.


## Test Batch Mode

At this point we should establish a hierarchy of information/file authority:

The scene/machine description in the .usda file is the primary source of truth.
A Klipper .cfg file and a ReprapFirmware config.gcode can be generated from a .usda file.

Mechanical features should be specified in the .usda file.
That means these things should be specified in the .usda file:
(+ means already there, M669/666 means translates into ReprapFirmware config value ...)

 - Motor microsteps,
 - Motor holdingTorque,
 - Motor numPolePairs, (implies motor full steps if stepper motor)
 - Motor dampingCoeff,
 + Motor rotational inertia,
 + Anchor positions,M669 ABCDIJKLO...
 - Mechanical advantage (numer of times line is routed back-and-forth between pivot/action points)
 - Gear ratios (one motor step might not move the spool one full step, idealized gears should be simulated)
 + Line stiffness/spring constant, M666 S...

The program that translates usda to cfg (`usda_to_cfg_and_configgcode.py`) should get some values as arguments, values that:

 - tune pure software features
 - Couple a microcontroller fw with its hw.

That means these things should be passed to the translator through arguments:

 * `step_pin`, `dir_pin`, `enable_pin` for each motor.
 * Spool buildup factor, M666 Q..., if any
 * Max/min/target planned forces, M666 I... X... T..., if any
 * Anchor Mode, M666 A...
 * Segments per second, M669 S... (probably best to set equal to timeCodesPerSecond in the .usda)

The `usda_to_cfg_and_configgcode.py` program should calculate from .usda and argument values:

 * Steps per millimeter or related settings local to the motor, including
   - Klipper's `rotation_distance` or similar settings for each motor
   - The Buildup Compensation feature affects what exectly these settings are called in Klipper
 * "Guy wire lengths" M666 Y... (similar to CablePath total length calculation)
 * Effector weight, M666 W... (sum of RigidGroup weights)
 * The number of anchors, M666 N...
 * Motor full steps, M666 J...

The number of defaults, and ECS values set in .js files, even if based on tags in the .usda file, should be minimal or purely related to rendering.
Otherwise such defaults will create bugs by hiding specified mechanical features and software features from the simulator.

A full test and its result depends on:

 - The simulation source code
 - The gcode
 - The .usda file that defines the machine
 - The .cfg/config.gcode file that was sent into klipper
 - Any bugs in the pacer or klipperCommander, if any
 - Any manual steps we have squeezed in (which we shouldn't)

A test batch is a collection of gcode files and usda files, maybe just bundled up in a common directory.
A `.usda_with_ranges` file is allowed to specify a parameter as a range,
to generate even more usda files (and hence .cfg/config.gcode files).
All pairs of (`gcode_n`, `usda_n`) are tested.

The output is a json with all quality measurements for each test run, ready to be plotted or optimized over.

This test batch mode is still something we can make in the web UI, with the json becoming a downloadable file,
and with possibility to re-run individual test runs to inspect them manually.


# Automated .usda Optimization

We want to eventually programmatically optimize our Hangprinter design, hw and sw features together.
Other than quantifying print quality, that means we need a new way to run print experiments.

 - Headless, no UI.
 - Fast locally, maybe one on every core of our cpu, or maybe on the gpu.
 - In a way that lets optimization algorithms like SLSQP optimize design parameters.

We can do this in 2D but we rather want to do it in 3D just to increase validity of our results.
Gravity and many other things change in 3D.


# 3D Simulation

We need to build out a 3d version of hp-sim5.

 - Cable joints library needs to be updated.
 - The UI needs to be updated.

It's needed for many purposes:

 - The hangprinter.org home page demos and other communication.
 - For simulating spool buildup (see `#Buildup_Compensation`).
 - To actually optimize the real HP5 (see `#Automated_usda_Optimization`).


# Misc

## Optimize Rendering by Caching

Make rendering faster. We always want faster rendering. Maybe by caching .pngs? when a run is finished?


## Optimize Performance and UX By Parallelizing

Parallellize more.
Each (individual machine of each) test run should run on a separate cpu, including each rendering process at the end.

We should be able to flip through each canvas individually and stack their (named and color identified) layers
in any order we want (drag & drop for ordering), similar to how layers work in graphics programs like Gimp and Inkscape.

