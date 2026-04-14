import numpy as np

# Config values should be based on your physical machine
guessed_anchors = np.array([[0.0, -2100 - (-100), 0.0],[1776 - 86.60254, 1025 - 50, 0.0],[-1776 - (-86.60254), 1025 - 50, 0.0]]) # Copied from slideprinter.usda and multiplied by 1000 m -> mm. Actually each spool position is subtracted respectively.
constant_spool_buildup_factor = 0.0043003 * 10  # Qualified first guess for 0.87 mm line
spool_r_in_origin_first_guess = np.array([30.0, 30.0, 30.0])  # Copied from slideprinter.usda and multiplied by 1000 m -> mm
spool_gear_teeth = 1 # No gears on slideprinter
motor_gear_teeth = 1 # No gears on slideprinter
mechanical_advantage = np.array([1.0, 1.0, 1.0])
lines_per_spool = np.array([1.0, 1.0, 1.0])
springKPerUnitLength = 20000.0
mover_weight = 2.0
