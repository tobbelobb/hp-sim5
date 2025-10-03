I have a question about the ASAP mode in hp-sim/index.html, and an idea about how to improve it.

# Background
hp-sim/index.html is part of a simulation app that reads input commands, coming either from .txt, .serial, or .gcode files, and generates move commands that get sent to a physics engine.
This physics engine tries to loop at a frequency of 500 Hz, that is 2 ms per time step, as specified in the slideprinter.usda file (in there it's called timeCodesPerSecond).
The physics engine and its time step loop was originally designed to run at real time but has since been augmented with a time scaling feature that is used by the hp-sim app via the ">> 2x" and ">> 0.5x" buttons, which speed up or slow down time inside
the physics loop, without altering the physics.

The app needs to synchronize the (potentially scaled) realtime of the physics loop with the emittance of Move commands. This is particularly challenging for the Klipper-generated commands found in the .txt and .serial files. These are `queue_step`
commands and contain information about timing in themselves. Here's what the klipper docs in ai_docs/Klipper_MCU_Commands.md says about `queue_step`:

```
queue_step oid=%c interval=%u count=%hu add=%hi : This command schedules 'count' number of steps for the given stepper, with 'interval' number of clock ticks between each step. The first step will be 'interval' number of clock ticks since the last
scheduled step for the given stepper. If 'add' is non-zero then the interval will be adjusted by 'add' amount after each step. This command appends the given interval/count/add sequence to a per-stepper queue. There may be hundreds of these sequences
queued during normal operation. New sequence are appended to the end of the queue and as each sequence completes its 'count' number of steps it is popped from the front of the queue. This system allows the micro-controller to queue potentially hundreds
of thousands of steps - all with reliable and predictable schedule times.
```

In it's context, "normal operation" would be having a host send these commands to a microcontroller. In our case, the "host" is the KlipperCommander class found in examples/js/slideprinter/klipperCommander.js. Our "microcontroller" is the physics
simulation itself, and in particular its the RemoteSpoolSystem class found in examples/js/slideprinter/slideprinter_common.js. The `queue_step` commands are translated by KlipperCommander to `Move` commands, which the RemoteSpoolSystem acts on right in
the physics simulation's time stepping loop.

# Improvement Idea
In ASAP mode, the user does not care if the physics simulation runs in a smooth linear realtime, or even a multiple of realtime. The user just wants Move commands for each time step calculated as fast as possible, and streamed to the simulators time
stepper as fast as possible. As for the time stepping loop, once the previous time step is finished, and the Move command from the .txt or .serial is available, there's no need to wait for anything, just do the time step as fast as possible.

Therefore, I want both the KlipperCommander and the `runGame` (including its internal `gameLoop` which handles the actuall call to `world.update(dt)`) function inside examples/js/slideprinter/runner.js to support a non-linear "ASAP-mode" in addition to
the current mode which we can call "linear-time-mode". If possible it would be nice to trigger ASAP-mode inside `runGame` whenever the time scale goes above a threshold value of 50. At that point a signal is sent to KlipperCommander to just fill the
buffer of Move commands as fast as possible, and they get processed as fast as possible. (Keep using the same buffers for both modes, just with different timing and synchronization strategies, as we want the ability to step in and out of ASAP-mode
seamlessly).

## The interaction with the related time scale feature
While in ASAP mode, the (linear) time scale should not be relevant, but we need to trigger the threshold timescale value so function calls like `gameControls.setTimeScale(asapThresholdScale);` and `handleTimeScaleChange(asapThresholdScale);` probably
should appear in the implementation, but the previous time scale before the ASAP-mode threshold was crossed should be restored whenever the print finishes. We should also support jumping back into linear-time mode if the user manually sets a time scale
below the ASAP threshold via `gameControls.setTimeScale(someLowValue);` or `handleTimeScaleChange(someLowValue);`.

There will be two ways to trigger the ASAP-mode via the UI.

 1: By clicking the finishAsapBtn.
 2: By manually increasing the timeScale my manually clicking the ">> 2x" button until the threshold is crossed.

In the asap-button case, the rendering should freeze until the print finishes. In the other case, the manual time scale increase case, the UI should still render, say every 10th frame or something, and go back to every frame once the print finishes.

# Files I expect you must touch
 - hp-sim/assets/hp-sim.js because the `finishAsapBtn` is defined there, along with `asapStatus` and `asapState`. Also functions such as `waitForAnimationFrames`, `runAsapFastForward`, `finalizeAsapMode`, and `triggerFinishAsap`.
 - examples/js/slideprinter/runner.js
 - examples/js/slideprinter/klipperCommander.js
 - examples/js/slideprinter/slideprinter_common.js because the RemoteSpoolSystem is touched by feature called `fast_mode` that might be affected or even useful for the implementation of the new ASAP mode.

# Open Questions
I see that the fast_mode feature in the RemoteSpoolSystem is enabled by the `triggerFinishAsap` function in hp-sim/assets/hp-sim.js. I don't know how much of the new ASAP-mode time synchronization idea is already implemented by fast_mode, but it looks
like the `gameLoop` is currently unaffected by fast mode.
