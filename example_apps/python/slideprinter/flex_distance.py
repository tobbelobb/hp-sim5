import numpy as np

# Axes indexing
A = 0
B = 1
C = 2
D = 3
I = 4
X = 0
Y = 1
Z = 2


def unit_vectors_along_force(anch_to_pos, distances):
    direction_vectors_for_each_position = np.ones(
        (np.size(anch_to_pos, 0), np.size(anch_to_pos, 2), np.size(anch_to_pos, 1))
    )
    if (abs(distances) > 0).all():
        direction_vectors_for_each_position = np.transpose(
            np.divide(anch_to_pos, distances[:, :, np.newaxis]), (0, 2, 1)
        )
    return direction_vectors_for_each_position


def forces_gravity_and_pretension(low_axis_max_force, low_axis_target_force, anch_to_pos, distances, mover_weight):
    mg = 9.81 * mover_weight

    force_directions = unit_vectors_along_force(anch_to_pos, distances)

    # Avoid division by zero
    threshold = 1e-8
    force_directions_z_safe = np.where(
        np.abs(force_directions[:, 2, 4]) > threshold, force_directions[:, 2, 4], threshold
    )

    top_mg = mg / force_directions_z_safe
    BCD_matrices = force_directions[:, :, (B, C, D)]
    ACD_matrices = force_directions[:, :, (A, C, D)]
    ABD_matrices = force_directions[:, :, (A, B, D)]
    ABC_matrices = force_directions[:, :, (A, B, C)]

    # Scale the top-direction vectors by the target_force
    top_pre = low_axis_target_force * force_directions[:, :, 4]
    top_grav = np.c_[force_directions[:, :3, 4] * top_mg[:, np.newaxis]] + np.array([0, 0, -mg])

    # Find the ABCD forces needed to cancel out target_force in top-direction
    BCD_forces_pre = np.zeros((np.size(distances, 0), 4))
    BCD_forces_grav = np.zeros((np.size(distances, 0), 4))
    ACD_forces_pre = np.zeros((np.size(distances, 0), 4))
    ACD_forces_grav = np.zeros((np.size(distances, 0), 4))
    ABD_forces_pre = np.zeros((np.size(distances, 0), 4))
    ABD_forces_grav = np.zeros((np.size(distances, 0), 4))
    ABC_forces_pre = np.zeros((np.size(distances, 0), 4))
    ABC_forces_grav = np.zeros((np.size(distances, 0), 4))
    try:
        BCD_forces_pre[:, (B, C, D)] = np.linalg.solve(BCD_matrices, -top_pre)
        BCD_forces_grav[:, (B, C, D)] = np.linalg.solve(BCD_matrices, -top_grav)
        ACD_forces_pre[:, (A, C, D)] = np.linalg.solve(ACD_matrices, -top_pre)
        ACD_forces_grav[:, (A, C, D)] = np.linalg.solve(ACD_matrices, -top_grav)
        ABD_forces_pre[:, (A, B, D)] = np.linalg.solve(ABD_matrices, -top_pre)
        ABD_forces_grav[:, (A, B, D)] = np.linalg.solve(ABD_matrices, -top_grav)
        ABC_forces_pre[:, (A, B, C)] = np.linalg.solve(ABC_matrices, -top_pre)
        ABC_forces_grav[:, (A, B, C)] = np.linalg.solve(ABC_matrices, -top_grav)
    except:
        pass

    # Just avoid dividing by zero
    threshold = 1e-8
    BCD_norms = np.linalg.norm(BCD_forces_pre, axis=1)[:, np.newaxis]
    BCD_norms = np.where(BCD_norms > threshold, BCD_norms, threshold)
    ACD_norms = np.linalg.norm(ACD_forces_pre, axis=1)[:, np.newaxis]
    ACD_norms = np.where(ACD_norms > threshold, ACD_norms, threshold)
    ABD_norms = np.linalg.norm(ABD_forces_pre, axis=1)[:, np.newaxis]
    ABD_norms = np.where(ABD_norms > threshold, ABD_norms, threshold)
    ABC_norms = np.linalg.norm(ABC_forces_pre, axis=1)[:, np.newaxis]
    ABC_norms = np.where(ABC_norms > threshold, ABC_norms, threshold)

    BCD_forces_pre = low_axis_target_force * BCD_forces_pre / BCD_norms
    ACD_forces_pre = low_axis_target_force * ACD_forces_pre / ACD_norms
    ABD_forces_pre = low_axis_target_force * ABD_forces_pre / ABD_norms
    ABC_forces_pre = low_axis_target_force * ABC_forces_pre / ABC_norms
    BCD_forces_grav = BCD_forces_grav / 4.0
    ACD_forces_grav = ACD_forces_grav / 4.0
    ABD_forces_grav = ABD_forces_grav / 4.0
    ABC_forces_grav = ABC_forces_grav / 4.0

    p = BCD_forces_pre + ACD_forces_pre + ABD_forces_pre + ABC_forces_pre
    m = BCD_forces_grav + ACD_forces_grav + ABD_forces_grav + ABC_forces_grav
    forces = [p, np.linalg.norm(top_pre, axis=1), m, np.linalg.norm(top_grav, axis=1)]

    return forces


def forces_gravity_and_pretension_scaled(
    low_axis_max_force, low_axis_target_force, anch_to_pos, distances, mover_weight
):

    [low_forces_pre, top_forces_pre, low_forces_grav, top_forces_grav] = forces_gravity_and_pretension(
        low_axis_max_force, low_axis_target_force, anch_to_pos, distances, mover_weight
    )

    # Avoid division by zero
    threshold = 1e-8
    pre = np.c_[low_forces_pre, top_forces_pre]
    pre_safe = np.where(pre > threshold, pre, threshold)

    # Make ABC_axes pull with exactly max force, or less
    scale_it = np.min(
        np.c_[
            np.max(
                abs((low_axis_target_force - np.c_[low_forces_grav, top_forces_grav]) / pre_safe),
                1,
            ),
            np.min(
                np.abs((low_axis_max_force - np.c_[low_forces_grav, top_forces_grav]) / pre_safe),
                1,
            ),
        ],
        1,
    )

    low_forces_pre = low_forces_pre * scale_it[:, np.newaxis]
    top_forces_pre = top_forces_pre * scale_it

    forces = np.c_[low_forces_pre + low_forces_grav, top_forces_pre + top_forces_grav]
    # forces = np.c_[low_forces_grav, top_forces_grav]
    # forces = np.c_[low_forces_pre, top_forces_pre]
    # forces = np.clip(forces, 0, np.max(forces))

    return forces


def flex_distance(
    low_axis_max_force, low_axis_target_force, anchors, pos, mechanical_advantage, springKPerUnitLength, mover_weight
):
    guyWireLengths = np.array(
        [
            np.linalg.norm(anchors[A] - anchors[I]),
            np.linalg.norm(anchors[B] - anchors[I]),
            np.linalg.norm(anchors[C] - anchors[I]),
            np.linalg.norm(anchors[D] - anchors[I]),
            100.0,
        ]
    )
    # Insert the origin as the first position always
    # It will be used for computing relative effects of flex later
    pos_w_origin = np.r_[[[0.0, 0.0, 0.0]], pos]
    anch_to_pos = anchors - pos_w_origin[:, np.newaxis, :]
    distances = np.linalg.norm(anch_to_pos, 2, 2)

    forces = forces_gravity_and_pretension_scaled(
        low_axis_max_force, low_axis_target_force, anch_to_pos, distances, mover_weight
    )

    springKs = springKPerUnitLength / (distances * mechanical_advantage + guyWireLengths)
    relaxed_spring_lengths = distances - forces / (springKs * mechanical_advantage)

    line_pos = relaxed_spring_lengths - relaxed_spring_lengths[0]

    distance_differences = distances - distances[0]
    impact_of_spring_model = line_pos - distance_differences

    return impact_of_spring_model[1:]
