"""Bound only regression dataset scheduling; preserve calibration behavior."""
import concurrent.futures
import os
import sys

if sys.argv[0].endswith("regress_calibration_logs.py"):
    _original_executor = concurrent.futures.ThreadPoolExecutor

    class _BoundedDatasetExecutor(_original_executor):
        def __init__(self, max_workers=None, *args, **kwargs):
            limit = int(os.environ.get("AUTOCAL_REGRESSION_MAX_WORKERS", "2"))
            super().__init__(min(max_workers or limit, limit), *args, **kwargs)

    concurrent.futures.ThreadPoolExecutor = _BoundedDatasetExecutor
