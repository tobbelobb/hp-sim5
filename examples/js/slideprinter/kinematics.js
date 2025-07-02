import {
    spool_r_in_origin_first_guess,
    spool_gear_teeth,
    motor_gear_teeth,
    mechanical_advantage,
    lines_per_spool,
    constant_spool_buildup_factor
} from './guessedData.js';
export { spool_r_in_origin_first_guess };

// --- Math Helpers ---
export function norm(arr) {
    return Math.sqrt(arr.reduce((sum, val) => sum + val * val, 0));
}
export function subtract(a, b) {
    return a.map((val, i) => val - b[i]);
}
export function add(a, b) {
    return a.map((val, i) => val + b[i]);
}
export function scale(a, s) {
    return a.map(val => val * s);
}

function distance(p1, p2) {
    return Math.hypot(p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]);
}

function distance_samples_relative_to_origin(anchors, pos) {
    const origin = [0.0, 0.0, 0.0];
    const dists_to_origin = anchors.map(anchor => distance(anchor, origin));

    const relative_line_lengths = pos.map(p => {
        const dists_to_p = anchors.map(anchor => distance(anchor, p));
        return dists_to_p.map((d, i) => d - dists_to_origin[i]);
    });

    return relative_line_lengths;
}

export function pos_to_motor_pos_samples_deg(
    anchors,
    pos,
    low_axis_max_force, // unused because use_flex is false
    use_flex, // unused because use_flex is false
    spool_buildup_factor = constant_spool_buildup_factor,
    spool_r_in_origin = spool_r_in_origin_first_guess
) {
    const spool_to_motor_gearing_factor = spool_gear_teeth / motor_gear_teeth;
    const mech_adv_ = mechanical_advantage;
    const lines_per_spool_ = lines_per_spool;

    // Buildup per line times lines. Minus sign because more line in air means less line on spool
    const k2 = mech_adv_.map((ma, i) => -1.0 * ma * lines_per_spool_[i] * spool_buildup_factor);

    // we now want to use degrees instead of steps as unit of rotation
    const degrees_per_unit_times_r = mech_adv_.map(ma => (spool_to_motor_gearing_factor * ma * 360.0) / (2.0 * Math.PI));

    const relative_line_lengths = distance_samples_relative_to_origin(anchors, pos);
    // flex_distance is not implemented as use_flex is false in MoveCommander

    if (k2.every(val => val > 1e-9)) {
        const spool_r_in_origin_sq = spool_r_in_origin.map(r => r * r);
        const k0 = degrees_per_unit_times_r.map((d, i) => 2.0 * d / k2[i]);
        return relative_line_lengths.map(lengths => {
            return lengths.map((len, i) => {
                return k0[i] * (Math.sqrt(Math.abs(spool_r_in_origin_sq[i] + len * k2[i])) - spool_r_in_origin[i]);
            });
        });
    } else {
        // Simplified kinematics without spool buildup (linear relationship)
        return relative_line_lengths.map(lengths => {
            return lengths.map((len, i) => {
                return (degrees_per_unit_times_r[i] / spool_r_in_origin[i]) * len;
            });
        });
    }
}
