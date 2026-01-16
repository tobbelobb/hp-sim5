# Find the mapping from Force Space to the Motor Space

When it starts up the first time, Hangprinter is an uncalibrated, overconstrained, translational cable driven parallel robot.

That means that its end effector is attached to anchors, with lines, from more than three directions.
Also, it doen't know where these lines start and where they end.
So it doesn't know where the end effector is or where the anchors are.

The most basic Hangprinter has no way of finding out.
A human must somehow find the anchor positions and manually take the end effector to a known position.

The smallest possible addition that could make a Hangprinter self-calibrating is by adding
 - A motor control mode, "force mode" that lets us tighten lines with an approximate amount of force, and
 - Encoders to its motors.

Force mode enables us to move around.
Encoders enables us to measure our movements.

# Move Around, Bottom Up Approach

For Hangprinter, "move around" can mean many things.
Let's look at a bottom-up hierarchy of what moving around can mean, and how we might map from one layer to the next,
and what approximations we accumulate along the way.

 0. Variations of "Force".
    1. We can typically apply a known amount of current through a motor. This amperage is usually well known and controlled, and no approxiamtion.
    2. The current creates an approximate torque in the motor. This torque can be very uneven due to friction or motor cogging.
       Let's call the unevenness "Approximation 0B", or the "Torque Approximation".
       Some drivers manage to measure and mitigate the unevenness, and make the Torque Approximation really small.
    3. The torque creates an amount of tension in the connected line. It can be more or less complicated to map between torque and tension force.
 1. Motor movement, denoted as radians, degrees, or encoder ticks per motor. This is what we measure with encoders.
    The list of motor positions is sometimes called Motor Space, Joint Space, or Actuator Space.
 2. Variations of "Line Length".
    Lines are getting wound in or out, denoted in millimeters of line length per anchor.
    All movements can be "relative" or "absolute" but for line lengths it plays a big role.
    Also, lines can get longer or shorter simply by pulling them.
    This is called flex or elongation.
    We want lines to elongate beacuse that means they are tight and the end effector is put under tension.
    This is for many reasons, not the least for counteracting gravitational forces that will elongate lines anyways.
    All in all we end up with 3 "ways to move around" in line space:
    1. Relative line lenghts. Aka relative line positions. This is how much shorter or longer the line is, compared to some unknown, but known to be constant, reference length.
       A "line movement" changes the absolute and relative line position equally as much.
       The difference is just if we know how far away we are from the anchor or not.
    2. Absolute line lenghts. Aka absolute line positions. This is simply the distance between the end effector and the anchor.
    3. Distance between end effector and anchor. Such a distance is in general a little longer than the corresponding absolute line length.
       This is because every tight line is a little bit elongated.
 3. Movement of the end effector, denoted in millimeters from the origin points along the Cartesian X, Y, and Z axes.
    The Cartesian space is sometimes called the Tool Space, End-effector Space, or Task Space.

Getting from type 0.3 to type 1 already introduces large approximations.
Just because we know the applied current, or even the motor torque, does not mean we know the line tightness.
The amount of force we apply in force mode is not exact. We don't know the friction and unevenness in the system.
There are always unknowns in the motion system, such as parts flexing, friction acting in unpredictable patterns, or parts being not perfectly round or straight.
We call the unknowns in the mapping from the commanded force to the real tightness in lines "Approximation B1", or "Force Approximation".

Getting from type 1 to type 2 also introduces unceirtainty.
Just because we know how much the motor moved does not mean we know how how much line was subtracted from the volume between the end effector and the anchor, or even how much was wound onto the spool.
There might be an imprecise belt linkage between the motor and the spool, or the retracted line might be in an elongated state, or the elongation state of the line was already on the spool
might have changed before, during, or after the motor movement.
So we can only approximate how motor movements map to line movements.
Call this "Approximation B2", of "Line Position Approximation".
There's even some hysteresis included in the Line Position Approximation because line elongation that gets wound onto a spool tends to stay a bit stretched out due to
friction on the spool, so a bit of non-linear error is "stored" on the spool. Line slack can also get stored on a spool.

Getting from type 2.1 movements to 2.2 movements, from relative line lenghts to absolute line lenghts, is simply a matter of
adding a constant; namely the distance from the reference point to the anchor.
In reality we will only know the approximate positions of the anchors, so the distances to them will also be approximate.
Call this "Approximation B3", or the "Anchor Position Approximation".

Getting from type 2.2 movements to 2.3 movments, from absolute line lengths to absolute distances is possible,
at least if we know approximate tightness in the line.
We can model how much the lines elongate.
A simple linear spring model gets us a bit of the way there even though it's a crude approximation. Call it "Approximation B4", or "Spring Approximation".
It's not entirely given that we can read all the motors' force settings at the same time.
Some motors might be in position mode and the motor driver might not expose its Ampere setpoints or readings to external users.
This is also a point where Approximation 1, the Force Approximation might come back to bite us if the force readings are skewed or noisy.

Lastly comes the the mapping from absolute distances to Cartesian positions, from type 2.3 movements to type 3 movements.
It's a numerical algorithm that has to work well with the noisy and skewed data that Approximations 1-4 create.
Some error accumulates at this stage as well since no algorithm is perfect. Call that "Approximation B5", or "Numerical Approximation".

## What's Forward, what's Inverse?
In our hierarchy above we chose to list six different spaces/"lists of things" to keep track of.

In robotics terminology it's often just two, corresponding to our type 1 and type 3 movements.
Robot are usually commanded by sending Cartesian coordinates (our type 3 movements) to them.
These coordinates get sent to a Motion Solver, often called Inverse Kinematics or the Inverse Transform.

The Motion Solver then covers mappings all the way from 3 to 1, but not to type 0.
In robotics texts the mapping from motor position to motor current is often skipped because the motor driver handles that similarly on all robots.

The mapping from type 1 to type 3 is called the Pose Evaluation, Forward Kinematics, or Forward Transform.
A "Pose" is understood to mean "something in Cartesian space".
Forward is called "Forward" because people imagine the effects of input (often understood as motor movement) to travel "forward" through your robot or motion system.

Klipper and ReprapFirmware has a nice 1->3 forward transform based on [A Quadratic Approximation Approach](https://doi.org/10.1007/978-3-031-94608-0_1) by Mahnke
& Caverly, and is tested to work better than most algorithms on noisy and skewed data from a range of different machines.

Let's use "forward" to mean any mapping taking us from lower type numbers to higher type numbers.
So the bottom-up mappings we described above are all little forward sub-transforms, taking us from one space to the next.
We described which approximations are usually made in each little forward sub-transform.
These "approximations" also known as "error sources".

If it's possible to have a change in one list without getting a corresponding or predictable change in the next list in the hierarchy,
then we have an "approximation" or an "error source".
It simply means our physics model (our sequence of lists) is skipping phenomena or processes that are important to our real motion system.
So the approximations are holes in our physics model, missing forward sub-transforms.

An example: a movement in motor space, a type 1 movement, that is a change in the list of type 1 can occur without any line getting wound in or out
if a pair of gears simply absorbed the movement as backlash. Gear movement, and hence their potential backlash, is not covered by any of lists in our hierarchy/physics model.

The hierarchy of lists and how we choose to map between the lists is our physics model, also known as our "kinematics".

# Move Around, Top Down Approach

We're talking about this bottom-up movement hierarchy because we really want to traverse it in the opposite direction, we want a good Motion Solver, aka Inverse Kinematics.
We want to map Cartesian positions all the way down to motor currents, as perfectly as we possibly can.

Getting from 3 to 2.3 is called the "inverse transform", mapping a list of Cartesian xyz-values to a list of line lengths, or simply distances between the end effector
and each of the anchors.

(the motor drivers handle mapping motor positions to drive currents).


We want to know:
 - Given a list of line tightnesses, what motor positions and relative line positions can we expect?
