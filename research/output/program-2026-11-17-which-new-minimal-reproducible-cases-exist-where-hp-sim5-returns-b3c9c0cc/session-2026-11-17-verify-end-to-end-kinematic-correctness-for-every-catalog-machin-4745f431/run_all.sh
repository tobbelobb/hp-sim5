#!/usr/bin/env bash
# Run the full kinematic sweep, both bug repros and the negative-result control.
# The main sweep and both repros exit 1 while the findings are present.
set -u
cd "$(dirname "$0")"
mkdir -p logs

run() {
  local name="$1"; shift
  echo "=== $name"
  "$@" > "logs/$name.log" 2>&1
  local rc=$?
  echo "$name exit=$rc (log logs/$name.log)"
  return $rc
}

run main-5mm node kinematics_e2e.mjs --move=5 --settle=10000 --avg=4000 --out=results.json
run repro-layered node repro_layered_radius_scale.mjs
run repro-cubecorners node repro_cubecorners_tip_rotation.mjs
run control-other-axes node control_all_other_axes.mjs

echo
echo "Findings expected nonzero: main-5mm, repro-layered, repro-cubecorners."
echo "Control expected zero: control-other-axes."
echo "summary:"
for f in logs/*.log; do
  echo "--- $f"
  tail -n 4 "$f"
done
