"""Compare firmware anchor coordinates."""

import math
import re


def anchor_errors(initial, final):
    def parse(text):
        anchors = {}
        for name in 'ABCDIJKLO':
            match = re.search(rf'\b{name}\s*:?\s*([-+\d.eE]+)[,:]\s*([-+\d.eE]+)[,:]\s*([-+\d.eE]+)', text)
            if match:
                anchors[name] = [float(v) for v in match.groups()]
        return anchors
    truth, fitted = parse(initial), parse(final)
    if len(truth) < 4 or truth.keys() != fitted.keys():
        raise ValueError("missing anchor: initial and final firmware must contain the same complete anchor set")
    truth, fitted = list(truth.values()), list(fitted.values())
    diff = [[v - t for v, t in zip(a, b)] for a, b in zip(fitted, truth)]
    shift = [sum(row[i] for row in diff) / len(diff) for i in range(3)]
    aligned = [[v - shift[i] for i, v in enumerate(row)] for row in diff]
    return {'reference_mm': truth, 'fitted_mm': fitted, 'error_mm': diff,
            'mean_translation_mm': shift, 'aligned_error_mm': aligned,
            'anchor_distance_mm': [math.sqrt(sum(v * v for v in row)) for row in diff],
            'aligned_anchor_distance_mm': [math.sqrt(sum(v * v for v in row)) for row in aligned],
            'max_abs_coordinate_mm': max(abs(v) for row in diff for v in row),
            'max_aligned_abs_coordinate_mm': max(abs(v) for row in aligned for v in row)}
