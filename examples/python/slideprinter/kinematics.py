from .flex_distance import *
from .guessed_data import *
import numpy as np


def distance_samples_relative_to_origin(anchors, pos):
    """
    Calculates the change in line length to each anchor from origin to pos.
    """
    # The origin is implicitly at [0,0,0] in this calculation
    pos_w_origin = np.r_[[[0.0, 0.0, 0.0]], pos]
    anch_to_pos = anchors - pos_w_origin[:, np.newaxis, :]
    distances = np.linalg.norm(anch_to_pos, axis=2)
    # distances is shape (num_positions, num_anchors)
    # where num_positions is len(pos) + 1
    # distances[0] is distances to origin
    # distances[1:] is distances to points in pos
    return distances[1:] - distances[0]


def pos_to_motor_pos_samples(
    anchors,
    pos,
    low_axis_max_force,
    use_flex,
    spool_buildup_factor=constant_spool_buildup_factor,
    spool_r_in_origin=spool_r_in_origin_first_guess,
    spool_to_motor_gearing_factor=spool_gear_teeth / motor_gear_teeth,
    mech_adv_=mechanical_advantage,
    lines_per_spool_=lines_per_spool,
):
    """
    What motor positions (in degrees) motors would be at,
    given anchor and data collection positions.
    """

    # Assure np.array type
    spool_r_in_origin = np.array(spool_r_in_origin)
    mech_adv_ = np.array(mech_adv_)
    lines_per_spool_ = np.array(lines_per_spool_)

    spool_r_in_origin_sq = spool_r_in_origin * spool_r_in_origin

    # Buildup per line times lines. Minus sign because more line in air means less line on spool
    k2 = -1.0 * mech_adv_ * lines_per_spool_ * spool_buildup_factor

    # we now want to use degrees instead of steps as unit of rotation
    # so setting 360 where steps per motor rotation is in firmware buildup compensation algorithms
    degrees_per_unit_times_r = (spool_to_motor_gearing_factor * mech_adv_ * 360.0) / (2.0 * np.pi)

    relative_line_lengths = distance_samples_relative_to_origin(anchors, pos)
    if use_flex:
        relative_line_lengths += flex_distance(
            low_axis_max_force,
            np.max(np.array([low_axis_max_force - 1, 0.0001])),
            anchors,
            pos,
            mechanical_advantage,
            springKPerUnitLength,
            mover_weight,
        )

    if (k2 > 1e-9).all():
        k0 = 2.0 * degrees_per_unit_times_r / k2
        motor_positions = k0 * (np.sqrt(abs(spool_r_in_origin_sq + relative_line_lengths * k2)) - spool_r_in_origin)
    else:
        # Simplified kinematics without spool buildup (linear relationship)
        motor_positions = (degrees_per_unit_times_r / spool_r_in_origin) * relative_line_lengths

    return motor_positions
