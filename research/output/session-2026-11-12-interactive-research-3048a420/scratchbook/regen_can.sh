#!/usr/bin/env bash
# Regenerates the 20 bundled public/RRF_CAN_commands/*.can files with the
# RRF host simulator (run from the repo root after building rrf_simulator).
set -euo pipefail
out=$(mktemp -d)
for g in Hangprinter_logo6 draw_squares_bigger; do
  for m in cubecorners hp3 hp4 skycam slideprinter; do
    for variant in no_buildup w_line_layers; do
      cfg=config_$m.g; [[ $variant == w_line_layers ]] && cfg=config_${m}_w_line_layers.g
      name=${g}_${m}_${variant}
      ( ./RRF/build/rrf_simulator --vsd RRF/run/vsd --gcode gcodes/$g.gcode --can-log "$out/$name.csv" -c sys/$cfg > "$out/$name.log" 2>&1
        .venv/bin/python RRF/csv2can.py "$out/$name.csv" >> "$out/$name.log" 2>&1
        cp "$out/$name.can" public/RRF_CAN_commands/$name.can
        echo "$name $(grep 'Final machine' "$out/$name.log")" ) &
    done
  done
done
wait
echo "logs in $out"
