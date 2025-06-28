import numpy as np
import itertools

def slideprinter_forward_transform(anchors, line_lengths):
    """
    Calculates the position of the effector given the lengths of the lines
    connecting it to the anchors.

    Supports 3, 4, or 5+ anchors.
    - 3 anchors (triangular case / trilateration): The intersection of 3 spheres.
      Returns one of two solutions, assuming the effector is below the anchors.
    - 4 anchors (tetrahedron case): Solves a 3x3 linear system.
    - 5+ anchors (overdetermined case): Solves multiple 3x3 systems and
      averages the results.
    """
    num_anchors = anchors.shape[0]

    if num_anchors < 3:
        raise ValueError("At least 3 anchors are required.")

    if num_anchors == 3:
        # Trilateration for 3 anchors
        p1, p2, p3 = anchors[0], anchors[1], anchors[2]
        r1, r2, r3 = line_lengths[0], line_lengths[1], line_lengths[2]

        temp1 = p2 - p1
        e_x = temp1 / np.linalg.norm(temp1)
        temp2 = p3 - p1
        i = np.dot(e_x, temp2)
        temp3 = temp2 - i * e_x
        e_y = temp3 / np.linalg.norm(temp3)
        e_z = np.cross(e_x, e_y)
        d = np.linalg.norm(p2 - p1)
        j = np.dot(e_y, temp2)

        x = (r1**2 - r2**2 + d**2) / (2 * d)
        y = (r1**2 - r3**2 + i**2 + j**2) / (2 * j) - (i / j) * x
        
        z_sq = r1**2 - x**2 - y**2
        if z_sq < 0:
            # No real solution
            return None, float('inf')
        
        # Assume the solution is below the anchor plane.
        # The direction of e_z determines "up" or "down".
        # If anchors are roughly in a plane, e_z is normal to it.
        # A negative z in this new basis means it's on one side of the plane.
        z = -np.sqrt(z_sq)

        p = p1 + x * e_x + y * e_y + z * e_z
        return p, 0.0

    # For num_anchors >= 4, use linear system derived from subtracting sphere equations.
    # Use the last anchor as the reference.
    ref_anchor = anchors[-1:]
    other_anchors = anchors[:-1]

    anch_prim = other_anchors - ref_anchor

    norms_sq = np.sum(anchors * anchors, axis=1)
    l_sq = line_lengths * line_lengths

    k = ((norms_sq[:-1] - norms_sq[-1]) - (l_sq[:-1] - l_sq[-1])) / 2.0

    num_other_anchors = num_anchors - 1

    if num_other_anchors == 3:  # Exactly 4 anchors
        M = anch_prim
        if abs(np.linalg.det(M)) < 1e-6:
            return None, float('inf')
        p = np.linalg.solve(M, k)
        return p, 0.0

    # Overdetermined system (num_anchors > 4)
    p_solutions = []
    indices = list(range(num_other_anchors))

    # For 5 anchors (4 other anchors), we can form C(4,3)=4 subsystems.
    for subset_indices in itertools.combinations(indices, 3):
        M = anch_prim[list(subset_indices), :]
        k_subset = k[list(subset_indices)]
        if abs(np.linalg.det(M)) > 1e-6:
            p_sol = np.linalg.solve(M, k_subset)
            p_solutions.append(p_sol)

    if not p_solutions:
        return None, float('inf')

    p_solutions = np.array(p_solutions)
    p_avg = np.mean(p_solutions, axis=0)

    diff = p_solutions - p_avg
    spread = np.sum(diff * diff)

    return p_avg, spread
