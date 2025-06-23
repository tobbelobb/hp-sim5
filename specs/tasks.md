# Strategy 1: Warp first 2d track. Go all the way with Python/Warp Slideprinter (2d Hangprinter) before making Cable Joints 3d capable.
 - Make slideprinter.html with Python
 - Make the Slideprinter contained in a USD file
 - Visualize Slideprinter in 3d with Warp
 - Connect Slideprinter to control software, preferrably Klipper.
   The design of the control interface at this stage is really crucial.

# Strategy 2: Warp first 3d track. Make 3d capable Cable Joints in Python/Warp first.
 - Make and visualize a simple hanging pendulum in 3D with Warp. The pendulum could just hang in a standard distance constraint at first.
 - Replace the distance constraint with a Cable Joint constraint.
 - Visualize the pendulum swinging and bumping into another pendulum
 - Make a pendulum's cable hit and wrap around a cylinder during swing
 - Connect 4 cables to a sphere with hybrid-attachment, and have spool at the far end of each cable,
   so we can pull the sphere around like a Hangprinter if we could control the spools.
 - Add a simple scheme for controlling the spools.
 - Connect Python code to control the spools, through some control interface. Klipper or maybe
   auto-calibration-simulation-for-hangprinter code could control spools initially. Or even hard coded numbers.
   This interface is what a neural net or other RL trained AI would interface with in order to become
   AI control mechanism.

# Strategy 3: 3d Cable Joints first, Warp later track. Make 3d capable Cable Joints in Python but keep browser/JavaScript as a visualization platform at first. And then try to replicate with Warp.
 - Look at ten-minute-physics tutorial for how to use 3d js visualization library to visualize balls in 3d.
 - Extend cable joints code (in both js and python) from 2d to 3d.
 - Build a simple debug scene with pendulums in 3d just like we did in 2d.
 - Build some kind of 3d flipper?
