import { rrfMotorId } from './rrfFirmwareModel.js';

export function parseCanMotionRow(line) {
    const fields = line.split(',');
    if (fields.length < 7 || !/^\d+$/.test(fields[0])) {
        return [];
    }
    const [index, canAddress, whenToExecute, accelTicks, steadyTicks, decelTicks] =
        fields.slice(0, 6).map(Number);
    if (!Number.isInteger(canAddress) || canAddress < 0 || canAddress > 126
        || ![whenToExecute, accelTicks, steadyTicks, decelTicks].every(Number.isFinite)) {
        return [];
    }

    // The firmware writes perDrive[0..numDrivers) without compacting empty
    // slots. The final two columns are shared by all drivers in the message.
    // Older HTTP responses may omit the profile fields.
    const profileStart = Math.max(7, fields.length - 2);
    const values = fields.slice(6, profileStart);
    if (values.length > 8 || values.some(value => !value.trim() || !Number.isFinite(Number(value)))) {
        return [];
    }
    const acceleration = Number(fields[profileStart] ?? 0);
    const deceleration = Number(fields[profileStart + 1] ?? 0);
    return values.map((value, driver) => ({
        index,
        canAddress,
        driver,
        motorId: rrfMotorId(canAddress, driver),
        whenToExecute,
        accelTicks,
        steadyTicks,
        decelTicks,
        steps: Number(value),
        // CanCapture writes fixed-point decimals for extruder drives,
        // including when the amount happens to be a whole number.
        isExtruder: value.includes('.'),
        acceleration,
        deceleration,
    }));
}
