import numpy as np

# Config values should be based on your physical machine
constant_spool_buildup_factor = 0.0043003 * 10  # Qualified first guess for 0.87 mm line
spool_r_in_origin_first_guess = np.array([75.0, 75.0, 75.0, 75.0, 75.0])
spool_gear_teeth = 255
motor_gear_teeth = 20
mechanical_advantage = np.array([2.0, 2.0, 2.0, 2.0, 4.0])
lines_per_spool = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
springKPerUnitLength = 20000.0
mover_weight = 2.0
