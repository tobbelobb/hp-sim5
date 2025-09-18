// Config values should be based on your physical machine
// Copied from slideprinter.usda and multiplied by 1000 m -> mm. Actually each spool position is subtracted respectively.
export const guessed_anchors = [
    [0.0, -2000.0, 0.0],
    [1689.39746, 975.0, 0.0],
    [-1689.39746, 975.0, 0.0]
];
// Qualified first guess for 0.87 mm line
export const constant_spool_buildup_factor = 0.043003;
// Copied from slideprinter.usda and multiplied by 1000 m -> mm
export const spool_r_in_origin_first_guess = [30.0, 30.0, 30.0];
// No gears on slideprinter
export const spool_gear_teeth = 1;
// No gears on slideprinter
export const motor_gear_teeth = 1;
export const mechanical_advantage = [1.0, 1.0, 1.0];
export const lines_per_spool = [1.0, 1.0, 1.0];
export const springKPerUnitLength = 20000.0;
export const mover_weight = 2.0;
