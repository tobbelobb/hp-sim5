# Warp Documentation Summary

This document summarizes key points from the official [Warp documentation](https://nvidia.github.io/warp/) as of version 1.7.2. Warp is a Python framework for writing high-performance simulation and graphics code. Regular Python functions are JIT compiled to efficient CPU or GPU kernels. Warp targets spatial computing with primitives for physics simulation, perception, robotics, and geometry processing. Kernels are differentiable and can be used with PyTorch, JAX, or Paddle.

## Quickstart

Install Warp from PyPI:

```bash
pip install warp-lang
```

To install additional dependencies for examples and USD features:

```bash
pip install warp-lang[extras]
```

These wheels require CUDA 12 drivers (minimum 525.60.13 on Linux, 528.33 on Windows). For older drivers you can build from source or use nightly wheels.

## Running Examples

Warp's `examples` directory provides many simulation scripts. Run any example with:

```bash
python -m warp.examples.<example_subdir>.<example>
```

Most examples produce USD animations viewable in USD-compatible tools such as Omniverse, UsdView, or Blender. The packages `usd-core`, `matplotlib`, and `pyglet` are required for the examples; they are installed with the `warp-lang[extras]` option above.

See the official site for full documentation and additional examples.
