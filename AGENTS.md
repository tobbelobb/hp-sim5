Prefer small code changes and shallow stack traces.
Delete outdated code paths. Update tests to use the newest conventions.
Use local Python interpreter: .venv/bin/python

Before delivering changes to physics, motors, cable transport or scene dynamics,
run `npm run test:physics`. It rebuilds native/Wasm engines and checks the real
HP4 Print Logo motion at browser defaults plus loaded-state settling. A failed
stability check blocks delivery; startup, unit/parity checks and higher-substep
success do not replace it. Use headless production backends for experiments;
browser experiments should be rare and preferably performed by a human.

For simulation experiments, robot design and autocal research, follow
`research/AGENTS.md`. The `hp-sim5-research-agent` launcher connects native physics
tools and Rerun inspection to Codex; see `research/README.md`.
