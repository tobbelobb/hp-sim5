"""Exact range-equation sensitivity, not a physical simulator or hardware test."""
import json
import numpy as np
from autocal.home_compensation import compensate_home

a = np.array([[0., -1900., -280.], [1645., 950., -280.], [-1645., 950., -280.], [0., 0., 1900.]])
q = np.array([0.0, 0.0, -0.3])
k = np.array([0., 0., 0., 0.4])
eh = np.zeros(4)
em = np.linalg.norm(a, axis=1) + k + eh - np.linalg.norm(a-q, axis=1)
evidence = {"provenance": "synthetic exact", "home_extension_mm": eh, "measurement_extension_mm": em, "range_uncertainty_mm": 0.01}
results = {}
for label, scale in [('exact', 1.0), ('extension_60_percent', 0.6)]:
    data = dict(evidence, home_extension_mm=eh*scale, measurement_extension_mm=em*scale)
    try:
        _, predicted = compensate_home(a, k, data)
        results[label] = {"shift_mm": predicted.tolist(), "error_mm": float(np.linalg.norm(predicted-q))}
    except ValueError as exc:
        results[label] = {"rejected": str(exc)}
print(json.dumps({"anchors_mm": a.tolist(), "true_shift_mm": q.tolist(), "offset_mm": k.tolist(), "home_extension_mm": eh.tolist(), "measurement_extension_mm": em.tolist(), "results": results}, indent=2))
