Gcode dynamic input
Torque mode
Encoder readings
Maybe force sensing


We want to develop a script that defines the auto calibration procedure.
I've tried before

The challenges I've faces have been:
 1. The data is not collected automatically by the machine, which makes the procedure labor intensive.
 2. The data I've managed to collect has not been good enough, so I've had to fumble around with excluding the few data points that incur the largest cost, which is labor intensive and not automated.
 3. The cound anchor points need to be rotated after the procedure finishes, which can be a very hard and error prone process. I can't ask my users to actually do that themselves, it needs to be automated.
 4. If an anchor or some other parts of physical config for some reason changes, then the whole procedure (data collection, data pruning, optimization, gravity alignment) needs to be re-run.





Appendix: Establishing my own terminology about calibration procedures
-----------------------------------------

# The calibration loop

## Approach 1: One data point, one guess, iterations

 1. Assume we're at the starting point (0, 0) if 2d, or (0, 0, 20) if 3d.
 2. Assume all lines tight but not stretched lines, zero newtons of pull but no sag.
 3. Assume anchors are spread evenly or randomly around the origin, but assume no norms to be followed.
 4. Put all but 2 if 2D, 3 if 3D motors in torque mode, tell the rest to execute a G1 X10 F100.
 5. Measure the resulting position of the remaining motor(s).
 6. Go back to the starting point: G1 X0
 7. Exit torque mode with the remaining motors and force them back to the starting point as well.
 8. Goto 4.

A shorter way of putting it: Search for the anchor position vector `a`:
 1. Divide motors into `controlled motors` and `measuring motors`.
 2. Make a guess at `a` (based on data if you have data).
 3. Plan a move by controlled motors and make a prediction about what the measuring motors will measure.
 4. Make the move and the test the prediction.
 5. Control all motors back to the starting point.
 6. Goto 1.

## Approach 2: Many data points, then a huge optimization

 1. Divide motors into `controlled motors` and `measuring motors`.
 2. Plan ca 40 moves by controlled motors and collect data after each point with the measuring motors.
 3. Run a LS optimization over the data.

## Approach 3: Hybrid

Do approach 2 many times, so make many guesses like in approach 1 but base each guess on 3-10 data points, not just 1.

## As Pseudocode

We want to make larger and larger movement attempts.
Imagine we have a list of absolute coordinates: "Approach 1 coordinates" that we want to reach (list1),
and a separate list of relative coordinates: "Approach 2 coordinates" (list2).


Abstracted as functions:
guess(): Takes a guess at where the anchors are.
prepare(): Ensures all motors are where they were at the start, and the proper motors are in torque/measure mode.
movement(): executes a single G1.
measure(): Reads a single data point.

## Approach 1

guess()
for movement in list1 do:
  do:
    prepare()
    move()
    measure()
    guess()
  while (error > eps)

## Approach 2

guess()
prepare()
for movement in list2
  move()
  measure()
guess()

## Approach 3

guess()
for movement in list1 do:
  do:
    prepare()
    for movement in list2 do:
      movement()
      measure()
    guess()
  while (error > eps)



