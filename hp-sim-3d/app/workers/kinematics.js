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
    spool_r_in_origin = spool_r_in_origin_first_guess,
    spoolParams = {}
) {
    const value = (key, index, fallback) => spoolParams[key]?.[index] ?? fallback;
    const mech_adv_ = anchors.map((_, i) => value('U', i, mechanical_advantage[i] ?? 1));
    const lines_per_spool_ = anchors.map((_, i) => value('O', i, lines_per_spool[i] ?? 1));

    // Buildup per line times lines. Minus sign because more line in air means less line on spool
    const k2 = mech_adv_.map((ma, i) => -1.0 * ma * lines_per_spool_[i] * spool_buildup_factor);

    // we now want to use degrees instead of steps as unit of rotation
    const degrees_per_unit_times_r = mech_adv_.map((ma, i) => (value('H', i, spool_gear_teeth) / value('L', i, motor_gear_teeth) * ma * 360.0) / (2.0 * Math.PI));

    const relative_line_lengths = distance_samples_relative_to_origin(anchors, pos);
    // flex_distance is not implemented as use_flex is false in MoveCommander

    return relative_line_lengths.map(lengths => lengths.map((len, i) => {
        const radius = spool_r_in_origin[i];
        const radiusSquared = radius * radius + len * k2[i];
        if (radiusSquared < 0) throw new Error(`Spool ${i} has insufficient line for this move`);
        // Rationalized form also covers zero buildup on an individual axis
        // and avoids subtracting nearly equal radii for small moves.
        return 2 * degrees_per_unit_times_r[i] * len / (Math.sqrt(radiusSquared) + radius);
    }));
}
