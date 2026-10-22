# Verification

- `.venv/bin/python -m pytest -q tests/python/test_world_configuration.py tests/python/test_native_collector.py tests/python/test_research_agent.py tests/python/test_autocal_jobs.py tests/python/test_research_terminal.py tests/python/test_research_archive.py -m 'not slow'`: 102 passed, 14 deselected, 28.90 s.
- `.venv/bin/python -m pytest -q tests/python/test_world_configuration.py tests/python/test_research_agent.py::test_stdio_mcp_discovery_execution_readback_and_error -m ''`: 23 passed, 12.60 s; includes the real HP3 collection and lifecycle/source-reload checks.
- Four slow checks in test_native_collector/test_research_agent (real HP4 collection, cancellation, attachment/reconnect/dead supervisor, service-only shutdown): 4 passed, 62.49 s.
- `node node_modules/jest/bin/jest.js --runInBand tests/js/hp-sim-3d/headlessWorld.test.js tests/js/hp-sim-3d/researchControls.test.js`: 5 passed across 2 suites.
- `git diff --check`: passed.

Some checks overlap; counts are per command, not a sum of distinct tests. These are tooling/measurement checks, not calibration accuracy evidence.
