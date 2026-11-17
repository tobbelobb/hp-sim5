#!/usr/bin/env bash
# Run the whole investigation. Expected outcomes:
#   fuzz_invariants.mjs        exit 0  (51/51 invariants hold)
#   repro_rotate_zero_axis.mjs exit 1  (bug present)
#   repro_normalize_range.mjs  exit 1  (bug present)
#   emit + check_with_numpy_pxr exit 0 (JS agrees with pxr/numpy/scipy)
#   probe_limitations.mjs      exit 0  (measurements only)
#
# Everything reads only; no source file is modified.
set -u
cd "$(dirname "$0")"
REPO=$(cd ../../../.. && pwd)
mkdir -p logs

status=0

echo "== 1. invariant fuzz =="
node fuzz_invariants.mjs 2>&1 | tee logs/fuzz_invariants.txt | tail -3
rc=${PIPESTATUS[0]}
if [ "$rc" -ne 0 ]; then echo "UNEXPECTED: fuzz found a new invariant violation"; status=1; fi

echo
echo "== 2. repro: rotateAroundAxis with a zero axis =="
node repro_rotate_zero_axis.mjs > logs/repro_rotate_zero_axis.txt 2>&1
rc=$?
tail -2 logs/repro_rotate_zero_axis.txt
if [ "$rc" -eq 0 ]; then echo "UNEXPECTED: repro passes (bug fixed?)"; else echo "expected: repro exits $rc (bug present)"; fi

echo
echo "== 3. repro: normalize() at the double-range extremes =="
node repro_normalize_range.mjs > logs/repro_normalize_range.txt 2>&1
rc=$?
tail -2 logs/repro_normalize_range.txt
if [ "$rc" -eq 0 ]; then echo "UNEXPECTED: repro passes (bug fixed?)"; else echo "expected: repro exits $rc (bug present)"; fi

echo
echo "== 4. independent numpy / pxr / scipy reference =="
node emit_js_reference.mjs | tee logs/emit_js_reference.txt
"$REPO/.venv/bin/python" check_with_numpy_pxr.py 2>&1 | tee logs/check_with_numpy_pxr.txt | tail -4
rc=${PIPESTATUS[0]}
if [ "$rc" -ne 0 ]; then echo "UNEXPECTED: reference check failed"; status=1; fi

echo
echo "== 5. documented-limitation probe =="
node probe_limitations.mjs 2>&1 | tee logs/probe_limitations.txt > /dev/null
echo "wrote logs/probe_limitations.txt"

echo
if [ "$status" -eq 0 ]; then
  echo "OVERALL: as expected (2 latent defects reproduced, all other invariants and references pass)"
else
  echo "OVERALL: unexpected result, see above"
fi
exit "$status"
