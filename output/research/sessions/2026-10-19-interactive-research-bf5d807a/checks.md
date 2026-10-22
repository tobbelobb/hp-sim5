# Validation

- 98 JS tests / 23 suites passed: autocal collector behaviors and primitives, RRF live and recorded bridge checks, current-position hold regression.
- Python hold and legacy command-pipeline regressions: 2 passed.
- Native collector tests excluding smoke: 10 passed, 1 failed on read-only Warp cache, 5 deselected. The failed test passed with WARP_CACHE_PATH=/tmp/hp-sim5-force-warp.
- git diff --check passed.
- Physical evidence is production native simulation only. No hardware measurement or optimizer accuracy result.
- Experimental implementation remains incomplete: HP4 automatic size tuning rejects reversed travel; HP3 full bootstrap uses more simulation time than baseline.
