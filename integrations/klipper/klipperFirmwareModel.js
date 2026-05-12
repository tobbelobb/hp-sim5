export const STEP_PIN_AXIS_MAP = {
    'gpio0': 'A',
    'gpio3': 'B',
    'gpio6': 'C',
    'gpio9': 'D',
    'gpio12': 'I',
    'gpio15': 'J',
    'gpio18': 'L',
    'gpio21': 'O',
    'gpio24': 'E',
};

export const DEFAULT_AXIS_ORDER = ['A', 'B', 'C', 'D', 'E', 'I', 'J', 'L', 'O'];
export const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16); // 200 steps/rev, 16 microsteps
export const MCU_CLOCK_HZ_KLIPPER_HOST = 50_000_000;

const EXTRUDER_ROTATION_DISTANCE_MM = 33.5;
export const EXTRUDER_MM_PER_STEP_KLIPPER = EXTRUDER_ROTATION_DISTANCE_MM / (200 * 16);
