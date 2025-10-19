# Planning with S-curve acceleration

## Current system

### InitStandardMove
After populating the DDA:
  * Set endSpeed to zero because we assume (for now) there is no following move
  * If the previous move has not been committed and both this and the previous move are printing moves or both non-printing moves, then [make this provisionally a deceleration-only move and] set targetNextSpeed of the previous move to min(requestedSpeed, sqrt(2 * deceleration * distance), call DoLookahead on the previous move, then set startSpeed to prev->endSpeed; else set startSpeed to zero
  * Call RecalculateMove

### RecalculateMove
We have startSpeed, endSpeed, requestedSpeed, totalDistance, acceleration, deceleration.
  * Calculate the distance to accelerate startSpeed->reqSpeed and decelerate reqSpeed→endSpeed
  * If the total is less than totalDistance, it's an accelerate/steady/decelerate move. Calculate topSpeed. Remove any very short accel or decel segment by reducing topSpeed.
  * Else calculate the [square of the] top speed we can reach if it's an accelerate-decelerate move.
  * If that's lower than [square of] startSpeed to endSpeed, it's an accelerate-only or decelerate-only move. Calculate the needed acceleration or deceleration needed to make it fit, record an error if it's more than 2% above the maximum for this move.
  * Set the ‘canPauseAfter’ flag depending on endSpeed
  * Calculate and store the total move time
Note, this relies on the combination of initial values being feasible with at most a 2% increase in acceleration or deceleration.

### DoLookahead
We have startSpeed, endSpeed, requestedSpeed, totalDistance, acceleration, deceleration, targetNextSpeed (which is the target start speed of the next move, hence our target end speed).
We also need decelDistance to be set up so that isDecelerationMove can work. We also need topSpeed to be set up because we compare it with the requested speed.
This function works in two phases: going ‘up’ (current move goes from the latest move added through older moves) and ‘down’ (moving back towards the latest move added).
When going up:
  * Limit targetNextSpeed to the requestedSpeed (this move may have been commanded to be at a lower speed than the next move, or limited to a lower speed because of different axes moving)
  * If the current move already meets its top speed, call MatchSpeeds to reduce targetNextSpeed if necessary, then switch to going down
  * Else if the current move is a deceleration-only move and we can adjust the previous move and the previous move has a deceleration phase: call MatchSpeeds to reduce targetNextSpeed if necessary;  calculate the maximum start speed we can have that allows us to decelerate to targetNextSpeed; set the targetNextSpeed of the previous move to that; increase laDepth and move on to the previous move (i.e. still going up)
  * Else the current move is not a deceleration-only move deceleration-only and we can’t adjust the previous one. Flag lookahead underrun. Assuming that this remains a deceleration-only move, calculate the maximum start speed that we can achieve and limit targetNextSpeed to that, then switch to going down.
  * Else the current move does not reach its top speed but it isn’t a deceleration-only move. Calculate the maximum start speed that we could achieve if it became deceleration-only and limit targetNextSpeed to that, then switch to going down.
When going down:
  * If we haven’t only just stopped going up, then set our start speed to the end speed of the previous move. Compute the maximum start speed we can achieve if we make this a pure acceleration move. Limit targetNextSpeed to this value.
  * If targetNextSpeed < endSpeed (which it could be by a small amount because of rounding error) do nothing, else increase endSpeed to targetNextSpeed
  * Call RecalculateMove
  * If laDepth == 0 then quit, else decrement laDepth and move on to the next move

# Prioritising the extruder
One of the requirements of the new system is that we must prioritise matching the extruder speed and acceleration between moves that have extrusion. This means that the overall end speed of one move will not in general match the start speed of the next move when there is extrusion, because it’s just the extrusion component that will. So when “going down” we will not necessarily be setting the targetNextSpeed of the next DDA to the start speed of the current one.

We can probably handle this in function MatchSpeeds; except that when we add a new move the calculation of targetNextSpeed may need to be changed, unless we are certain that MatchSpeeds will reduce it as needed.

This is perhaps best done separately from and in advance of S-curve.

Do we need to introduce a variable to represent the ratio of the end speed of one move to the start speed of the next, based on the extrusion rates? Or calculate it each time?

## Debugging
We need a simulation mode in which the moves are planned but not executed; instead when they are committed they are output to debug. We should delay committing them until all previous committed moves have been output. We will have to ignore the real step times while doing this.

## S-curve system
With S-curve we need to match both the speed and the acceleration between moves. Problem: the maximum speed we can reach depends not only on the starting speed and acceleration, but also on what ending acceleration is permitted i.e. what starting acceleration the next move can tolerate. The amount of acceleration that the next move can tolerate depends on its starting speed.

When we add the first move, ending speed and ending acceleration are zero, as are starting speed and acceleration. We can calculate the move. RecalculateMove can be changed to do this.

When we add a subsequent move, the ending speed and acceleration will be zero. Dpending on the move length we have these possibilities:

  * If it is possible to decelerate to zero from the requested speed and zero acceleration within the length of the move. In this case we can request the previous move to end at our requested speed and zero acceleration.
  * If it is not possible to decelerate to zero from the requested speed and zero acceleration. If we the previous move has the same requested speed, we can calculate a motion profile that eventually reaches that speed, then in principle we can calculate where on that motion profile the current move would end. It might end within the initial increasing deceleration segment, within a constant deceleration segment (if there is one), or within the final decreasing deceleration segment. The first and third of these require solving a cubic equation.
  * If the previous move does not have the same requested speed as ours, it’s more complicated. If it has a higher requested speed then we can calculate the motion profile as before and where on it the new move ends, then if the end of new move ends at/below its requested speed on that profile, we can proceed as before. If the new move would end above its requested speed then the ideal is that the new move ends at its requested speed with some acceleration.

It looks as though we may need to iterate back and forth between the two moves to get a good match.

Another difference: previously, we didn’t need to look again at acceleration-only moves, because we never needed to alter their end speeds (we couldn’t increase them and we never needed to reduce them). Now, if an acceleration move ends with a lower acceleration than the maximum, we may want to adjust it by increasing its end acceleration and implicitly its end speed.

## Useful formulae
Distance travelled:

	s = t * u + t2 * ai /2 + t3 * j/6
	s = t * v - t2 * af /2 + t3 * j/6

Distance travelled when the start speed and acceleration are zero:

	s = t3 * j/6

Instantaneous acceleration:

	a = ai + t * j

Time limit for a segment with increasing acceleration and a maximum acceleration:

	tmax = (amax – ai)/j

Time limit for a segment with increasing acceleration when start acceleration is zero:

	tmax = amax/j

Instantaneous speed:

	v = u + t * ai + t2 * j/2

Distance to reach max acceleration (verified using wxMaxima):

	s = u *  amax/j + (amax2 /6 – amax * ai2/2 + ai3/3)/j2

Distance to reach max acceleration if start speed and acceleration are zero:

	s = amax3/6j2

Speed upon reaching max acceleration if start speed and acceleration are zero:

	v = ?

For an S-curve acceleration move which starts at speed u and acceleration a, spends time t1 accelerating with jerk j, then spends time t2 at constant acceleration, then spends time t1 reducing acceleration back to a:

	s = u * (2 * t1 + t2) + a(2 * t12 + 2 * t1 * t2 + ½ * t22) + j * (t13 + (3/2) * t12 * t2 + ½ * t1 * t22)
	v = u + a * (2 * t1 + t2) + j * (t1 * t2 + t12)

If u = a = 0:

	s = j * (t13 + (3/2) * t12 * t2 + ½ * t1 * t22) = j * t1 * (t1 * (t1 + 1.5 * t2) + 0.5 * t22)
	v = j * (t1 * t2 + t12) = j * t1 * (t1 + t2)

If also t2 = 0:

	s = j * t13 = v * t1
	v = j * t12

Given a fixed t1 and a desired distance s we can solve for t2 thus:

	t2 = -(3/2) * t1 + sqrt(t12/4 + 8 * s/(j * t1))

Note to the above: j * t1 is the acceleration during the constant-acceleration segment.

## Discriminant and number of solutions
Starting from:

s = t * u + t2 * a/2 + t3 * j/6

and rewriting it to:

t3 * j/6 +  t2 * a/2 + t * u – s = 0

The discriminant of this is:

-18 * j/6 * a/2 * u * s + 4 * a3/8 * s + a2/4 * u2 – 4 * j/6 * u3 – 27 * j2/36 * s2

which simplifies to:

-(3/2) * j * a * u * s + ½ * a3 * s + (¼) *  a2 * u2 – (2/3) * j *  u3 – (¾) * j2 * s2

As we are only interested in the sign and whether it is zero, we can multiply by 12:

-18 * j * a * u * s + 6 * a3 * s + 3 * a2 * u2 – 8 * j * u3 – 9 * j2 * s2

Check:

t3 * j +  t2 * a * 3 + t * u * 6 – s * 6 = 0

Discriminant:

-18 * j * 3 * a * u * s * 6 * 6 + 4 * 27 * 6 * a3 * s + 9 * a2 * 36 * u2 – 4 * j * u3 * 36 * 6 – 27 * j2 * 36 * s2

Divide by 36:

-18 * j * 3 * a * u * s  + 18 * a3 * s + 9 * a2 * u2 – 4 * 6 * j * u3 – 27 * j2 * s2

Divide by 3:

-18 * j * a * u * s + 6 * a3 * s + 3 * a2 * u2 – 8 * j * u3 – 9 * j2 * s2

If a = 0 then we have a depressed cubic:

t3 * j + t * u * 6 – s * 6 = 0

The discriminant is:

- (4 * (6 * u)3 + 27 * 36 * s2)

which simplifies to:

- (4 * 6 * 36 * u3 + 27 * 36 * s2)

Dividing by 3 * 36:

- (8 * u3 + 9 * s2)


## Re-implementing move melding and planning with S-curve acceleration

The first move added to an empty queue is straightforward to plan because it needs to start and end with zero speed and acceleration.

To add the next move we could consider a number of algorithms:

- First identify the lower if the requested speeds of the old and new moves (often they will be the same). Ideally we want to transition between the two moves at this speed.
- For each move added, we could pre-calculate the maximum speed with zero acceleration that it could start at.
When we add the second move there will be a number of cases:
- Requested speed is the same as for the first move, we can accelerate to this requested speed and zero acceleration in the move already added, and we can decelerate from requested speed and acceleration in the move to be added. This is the simplest case.


