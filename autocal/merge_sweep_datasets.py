#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from autocal._autocal_common import _load_json, _merge_sweep_datasets, _write_json


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Merge one or more sweep datasets into a single output file."
    )
    parser.add_argument("base_dataset", type=Path, help="Base dataset JSON")
    parser.add_argument("extra_datasets", nargs="+", type=Path, help="Additional dataset JSON files")
    parser.add_argument("-o", "--output", required=True, type=Path, help="Merged output dataset JSON")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    merged = _load_json(args.base_dataset)
    for path in args.extra_datasets:
        merged = _merge_sweep_datasets(merged, _load_json(path))
    _write_json(args.output, merged)
    sweeps = merged.get("sweeps", [])
    sweep_count = len(sweeps) if isinstance(sweeps, list) else 0
    print(f"; merged -> {args.output} sweeps={sweep_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
