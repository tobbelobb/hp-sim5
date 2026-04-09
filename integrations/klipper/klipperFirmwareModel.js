export const STEP_PIN_AXIS_MAP = {
    'gpiochip1/gpio0': 'A',
    'gpiochip1/gpio3': 'B',
    'gpiochip1/gpio6': 'C',
    'gpiochip1/gpio9': 'D',
    // Lacking I, J, K, L, O
};

export const M569_DRIVER_AXIS_MAP = new Map([
    [40, 'A'],
    [41, 'B'],
    [42, 'C'],
    [43, 'D'],
    [44, 'E'],
    [45, 'I'],
    [46, 'J'],
    [47, 'K'],
    [48, 'L'],
    [49, 'O'],
]);

export const DEFAULT_AXIS_ORDER = ['A', 'B', 'C', 'D', 'E'];
export const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16); // 200 steps/rev, 16 microsteps
export const MCU_CLOCK_HZ_KLIPPER_HOST = 50_000_000;

const EXTRUDER_ROTATION_DISTANCE_MM = 33.5;
export const EXTRUDER_MM_PER_STEP_KLIPPER = EXTRUDER_ROTATION_DISTANCE_MM / (200 * 16);
