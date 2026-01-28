#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from typing import Optional, Sequence

from autocal.active_calibrate import ellipse_cli


def main(argv: Optional[Sequence[str]] = None) -> int:
    return ellipse_cli(argv)


if __name__ == "__main__":
    raise SystemExit(main())
