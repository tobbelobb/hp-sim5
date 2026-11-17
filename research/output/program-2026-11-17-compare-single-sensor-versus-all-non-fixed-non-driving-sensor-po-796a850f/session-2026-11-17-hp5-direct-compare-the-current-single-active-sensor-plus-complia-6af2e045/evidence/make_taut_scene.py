"""Create an HP5-direct scene whose spool links start with their geometric free length.

The catalog scene stores 2 m of cable in each first (spool-to-anchor) link.  A
production pullout pays that reservoir out before measuring, which also moves
the spool through many turns.  For small controlled experiments we instead
start each link at its geometric length, so encoder angle and free length stay
in the same approximately linear spool regime.  Geometry and all other scene
properties are unchanged.
"""
from __future__ import annotations

import hashlib
import re
from pathlib import Path

ROOT = Path('/home/torbjorn/repos/hp-sim5')
SOURCE = ROOT / 'public/usd_scenes/hp5.usda'
OUT = Path(__file__).resolve().parent / 'hp5_taut.usda'
LOW = 0.4290687590584987
TOP = 0.35226513973982904

text = SOURCE.read_text()
# Each CablePath block contains exactly one stored array.  The block name
# decides whether it is a low (2-line) or top (4-line) axis.
blocks = re.split(r'(?=    def Xform "CablePath)', text)
out = []
for block in blocks:
    match = re.match(r'    def Xform "CablePath(I[0-3]|[A-D][LR])"', block)
    if match:
        value = TOP if match.group(1).startswith('I') else LOW
        block, n = re.subn(r'custom double\[\] cablePath:stored = \[[^\]]*\]',
                           f'custom double[] cablePath:stored = [{value}, 0, 0]', block)
        assert n == 1, match.group(1)
    out.append(block)
text = ''.join(out)
OUT.write_text(text)
print(OUT)
print('sha256', hashlib.sha256(OUT.read_bytes()).hexdigest())
print('source', SOURCE)
print('source_sha256', hashlib.sha256(SOURCE.read_bytes()).hexdigest())
