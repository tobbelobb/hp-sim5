/read ai_docs/CableJoints/CableJoints.md ai_docs/Smallsteps/Smallsteps.md ai_docs/PBDBodies/PBDBodies.md src/python/cable_joints/update_attachment_points.py src/python/cable_joints/cable_joints_components.py examples/python/flipper/server.py src/python/cable_joints/pbd_cable_constraint_solver.py

/add src/python/cable_joints/pbd_record_which_cable_joints_are_taut.py src/python/cable_joints/pbd_record_which_cable_joints_are_taut.py

# High-Level Goal
Add a post-solver Slack-Resolution pass to PBDCableConstraintSolver.
Its job is to undo solver-induced over-contraction in cable joints that should still be taut, while leaving legitimately slack cable joints alone.

## 1. Where the New Pass Sits in the Frame Loop
        # 5. POSITIONAL SOLVERS: Correct predicted positions to satisfy constraints.
        world.register_system(PBDRecordWhichCableJointsAreTaut) # <-- New
        if use_warp:
            from cable_joints_warp.cable_solver_warp import WarpCableConstraintSolver
            solver = WarpCableConstraintSolver(device)
        else:
        ...
            solver = PBDCableConstraintSolver()
        world.register_system(solver)
        world.register_system(PBDResolveCableOverCorrections) # <-- New
        world.register_system(PBDBallBorderCollisions())

## 2. Algorithm
In PBDRecordWhichJointsAreTaut we simply record for each cable joint in the whole simulation (not per cable path in this case), if distance between attachment points were larger than the cable joint's rest length before the cable constraint solver step (in which case we record that the cable joint was taut), or if it was slack.

The PBDRecordWhichJointsAreTaut system can simply update an array of booleans found in a CableConstraintSolverCacheComponent which we can define in cable_joints.cable_joints_components.

Then, in the PBDResolveCableOverCorrections system we check if any of the cable joints that were previously taut have become slack.
Any such slackness is considered invalid so we want to resolve it.

Assume such slackness comes from multiple previously taut cable joints pulling on the same cable link object which creates an amplified and too large combined
effect.

Identify which cable link has been pulled, and which cable joints pulled it.
A cable link is considered over-projected if one of its attached cable joints have gone from taut to slack.

Use the positions previously recorded by the CableAttachmentCacheSystem to build an array of unit length direction vectors pointing from the previous cable link positions
to their cable attachment points. Call it `pull_directions` or something.

Find new target position to solve positional over correction for for each over-projected body. Bring all its incident joints back to rest length simultaneously (or as close as we can get to such a point),
but travel only along positive directions of `pull_directions` (I think this requires some translation between coordinate systems, try to make it simple though).

Apply correction PBD style, mass-weighted. (The position in both ends of each cable joint potentially needs to be adjusted.)

Repeat for all over-projected bodies
