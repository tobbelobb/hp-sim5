# Find the mapping from force space to the inverse kinematics space

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

 0. Applying a known amount of current through a motor, creating an approximate amount of tension in the connected line.
 1. Motor movement, denoted as radians or degrees per motor. This is what we measure with encoders.
 2. Lines getting longer or shorter, denoted in millimeters of line length per anchor. The list of line lengths is called "the inverse space", and changes to that list is
    sometimes called "movements in the inverse space". All movements can be "relative" or "absolute" but for line lengths it plays a big role:
    1. Relative line lenghts. Aka relative line positions. This is how much shorter or longer the line is, compared to some unknown, but known to be constant, reference length.
       A "line movement" changes the absolute and relative line position equally as much.
       The difference is just if we know how far away we are from the anchor or not.
    2. Absolute line lenghts. Aka absolute line positions. This is simply the distance between the end effector and the anchor.
 3. Movement of the end effector, denoted in millimeters from the origin points along the Cartesian X, Y, and Z axes. The Cartesian space is sometimes called the "forward space".

Getting from type 0 to type 1 already introduces large approximations.
Just because we know the applied current, or even the motor torque, does not mean we know the line tightness.
The amount of force we apply in force mode is not exact. We don't know the friction and unevenness in the system.
There are always unknowns in the motion system, such as parts flexing, friction acting in unpredictable patterns, or parts being not perfectly round or straight.
We call the unknowns in the mapping from the commanded force to the real tightness in lines "Approximation B1", or "Force Approximation".

Getting from type 1 to type 2 also introduces unceirtainty.
Just because we know how much the motor moved does not mean we know how how much the line moved.
The unknowns that created the Force Approximation also makes the relationship between motor position and line position imperfect.
So we can only approximate how motor movements map to line movements.
Call this "Approximation B2", of "Line Position Approximation".

Getting from type 2.1 movements to 2.2 movements, from relative line lenghts to absolute line lenghts, is simply a matter of
adding a constant; namely the distance from the reference point to the anchor.
In reality we will only know the approximate positions of the anchors, so the distances to them will also be approximate.
Call this "Approximation B3", or the "Anchor Position Approximation".

Lastly comes the the mapping from inverse space to forward space, from type 2.2 movements to type 3 movements, from absolute line lengths to a Cartesian position.
This step is called a "forward transform".
It's a numerical algorithm that has to work well with the noisy and skewed data that Approximations 1-3 create.
Some error accumulates at this stage as well since no algorithm is perfect. Call that "Approximation B4", or "Numerical Approximation".
Klipper and ReprapFirmware has a nice forward transform that is tested to work better than most algorithms
on noisy and skewed data from a range of different machines.


# Move Around, Top Down Approach



