Look at cable_joints/cable_joints_core.js, and the New Attachment Points logic. I don't understand how this line of code can work:
```
  const pADiffFromRotation = attachmentA_previous.clone().rotate(deltaAngleA, prevPosA, true).subtract(attachmentA_previous);
```
The prevAngleA is not the same angle as was used last time attachmentPointA_world was calculated (see ordering of systems registration in flipper.html).
prevAngleA is the angle we got after the previous frame's PDE solvers had run.
Yet, we use prevAngleA (or rather, orientationAComp.prevAngle) seemingly to calculate how much we should rotate attachmentPointA_world around the ball's position (its center), compared to how much we rotated it the previous time we calculated attachmentPointA_world? AHA! Maybe this works because the attachment should simply rotate together with the ball, regardless of the reason why the ball is rotating.
Temporary rotations (mid time step, for example after some system but before solver system) should not affect where this attachment is placed since pure rotation does not change the angle to the tangent point between a circle and a point, or between two circles.
It is tricky to understand because the analogous line for translational motion, `const pADiffFromTranslation = posA.clone().subtract(prevPosA)` needs prevPosA to represent the previous position we used when we calculated the previous attachmentPointA_world.
We need this because a translation does affect tangents.
We need to keep track of sA and sB, who in turn update the joint.restLength and path.stored values who are affected by the amount of winding/unwinding of cable due to ball across it's cable direction, from one tangent point to another, potentially in both ends of the cable.
So CableAttachmentUpdateSystem needs the intermediate prevPos, captured exactly when the previous attachmentPointA_world and attachmentPointB_world were calculated.
However, I want the PrevStateSystem to be the first that runs in every time step, and I don't want to capture an intermediate prevPos inside of CableAttachmentUpdateSystem.
I know this starts to sound theoretically impossible but please suggest three ways I could pull magic tricks or make compromises on my desires in order to make this work the way I want, or closer to the way I want.
