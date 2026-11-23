#!/bin/bash
# wait until no e4_live_hp3 process runs
while pgrep -f e4_live_hp3.py >/dev/null; do sleep 15; done
