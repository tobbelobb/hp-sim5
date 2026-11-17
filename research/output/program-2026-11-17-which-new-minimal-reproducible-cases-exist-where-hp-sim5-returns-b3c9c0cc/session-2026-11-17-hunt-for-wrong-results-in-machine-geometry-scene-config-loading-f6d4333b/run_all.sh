#!/usr/bin/env bash
# Reproduce all findings and control experiments.  Repro scripts exit nonzero
# when their bug is present; control scripts exit zero when the expected
# (correct) behaviour holds.
set -u
cd "$(dirname "$0")"
mkdir -p logs
PY=/home/torbjorn/repos/hp-sim5/.venv/bin/python

run() {
  local name="$1"; shift
  echo "=== $name ==="
  "$@" >"logs/$name.log" 2>&1
  local status=$?
  echo "exit=$status  (logs/$name.log)"
  tail -n 6 "logs/$name.log"
  echo
  return 0
}

run 1_xform_order        "$PY" repro_xform_order.py
run 1b_xform_minimal     node repro_xform_order_minimal.mjs
run 2_missing_attr       node repro_missing_attr.mjs
run 3_meters_per_unit    node repro_meters_per_unit.mjs
run 4_noise_units        "$PY" repro_noise_units.py
run 5_forward_roundtrip  "$PY" probe_forward_roundtrip.py
run 6_catalog_checks     "$PY" check_catalog_consistency.py
run 7_scene_loads        node probe_scenes.mjs
run 8_transform_audit    "$PY" audit_transforms.py
run 9_forward_tests      "$PY" /home/torbjorn/repos/hp-sim5/forward-transform-dev/hangprinter-forward-transform/test_forward.py
run 10_python_geometry   "$PY" -m pytest -q /home/torbjorn/repos/hp-sim5/tests/python/test_hp5_geometry.py /home/torbjorn/repos/hp-sim5/tests/python/test_machine_catalog.py
