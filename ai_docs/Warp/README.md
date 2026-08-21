# Warp Documentation Summary

This document summarizes the Warp concepts used by hp-sim5. The repository's
current dependency floor is `warp-lang>=1.12.0`, declared in
`requirements-warp.txt`. Consult the official [Warp
documentation](https://nvidia.github.io/warp/) for version-specific platform,
driver, and API details.

Warp is a Python framework for writing high-performance simulation and graphics
code. Regular Python functions are JIT compiled to efficient CPU or GPU kernels.

## Quickstart

Install the repository's core and Warp dependencies into its local environment:

```bash
.venv/bin/python -m pip install -r requirements-warp.txt
```

The hp-sim5 Warp cable solver lives in
`src/python/cable_joints_warp/cable_solver_warp.py`. Run its exercised Flipper
path on CPU with:

```bash
.venv/bin/python -m example_apps.python.flipper.server --warp --device cpu
```

Use `--device cuda:0` instead when a compatible CUDA device and driver are
available.

## Running Examples

Warp's own installed examples can be run with:

```bash
.venv/bin/python -m warp.examples.<example_subdir>.<example>
```

Those upstream examples can have dependencies beyond hp-sim5's
`requirements-warp.txt`; follow the documentation for the installed Warp
version when running them.
