"""3D Warp visualization of the Slideprinter.

This script mirrors the 2D Slideprinter demo from ``slideprinter_server.py``
but outputs a 3D USD animation using Warp's simulation tools.  It builds a
simple system of three moving spools, each tethered to a static anchor.  The
spools are also connected to each other with distance springs to maintain a
triangular structure.

Run with ``python -m python.slideprinter_warp``.  The generated ``slideprinter.usd``
file can be opened in any USD viewer.  See ``ai_docs/Warp/README.md`` for
Warp installation instructions.
"""

import math
import warp as wp
import warp.sim
import warp.sim.render


class SlideprinterExample:
    def __init__(self, stage_path: str = "slideprinter.usd"):
        self.frame_dt = 1.0 / 60.0
        self.substeps = 10
        self.dt = self.frame_dt / self.substeps
        self.time = 0.0

        builder = wp.sim.ModelBuilder()

        spool_radius = 0.03
        spool_mass = 0.005
        anchor_mass = 0.0
        dist = 0.1

        configs = [
            {
                "pos": (0.0, -dist, 0.0),
                "vel": (1.0, 0.0, 0.0),
                "anchor": (0.0, -dist - 2.0, 0.0),
            },
            {
                "pos": (
                    dist * math.cos(math.pi / 6),
                    dist * math.sin(math.pi / 6),
                    0.0,
                ),
                "vel": (-1.0 / math.sqrt(2), 1.0 / math.sqrt(2), 0.0),
                "anchor": (
                    2.05 * math.cos(math.pi / 6),
                    2.05 * math.sin(math.pi / 6),
                    0.0,
                ),
            },
            {
                "pos": (
                    dist * math.cos(5 * math.pi / 6),
                    dist * math.sin(5 * math.pi / 6),
                    0.0,
                ),
                "vel": (0.0, 0.0, 0.0),
                "anchor": (
                    2.05 * math.cos(5 * math.pi / 6),
                    2.05 * math.sin(5 * math.pi / 6),
                    0.0,
                ),
            },
        ]

        self.anchors = []
        self.spools = []

        for cfg in configs:
            a = builder.add_particle(
                wp.vec3(*cfg["anchor"]), wp.vec3(0.0, 0.0, 0.0), anchor_mass
            )
            self.anchors.append(a)

        for cfg in configs:
            s = builder.add_particle(
                wp.vec3(*cfg["pos"]), wp.vec3(*cfg["vel"]), spool_mass, radius=spool_radius
            )
            self.spools.append(s)

        for i in range(3):
            builder.add_spring(
                self.anchors[i], self.spools[i], ke=2.0e4, kd=0.0, control=0.0
            )
        builder.add_spring(self.spools[0], self.spools[1], ke=1.0e4, kd=0.0, control=0.0)
        builder.add_spring(self.spools[1], self.spools[2], ke=1.0e4, kd=0.0, control=0.0)
        builder.add_spring(self.spools[2], self.spools[0], ke=1.0e4, kd=0.0, control=0.0)

        self.model = builder.finalize()
        self.model.ground = False

        self.integrator = wp.sim.XPBDIntegrator()
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()

        self.renderer = wp.sim.render.SimRenderer(self.model, stage_path, scaling=10.0)

        self.use_cuda_graph = wp.get_device().is_cuda and wp.is_mempool_enabled(
            wp.get_device()
        )
        if self.use_cuda_graph:
            with wp.ScopedCapture() as capture:
                self.simulate()
            self.graph = capture.graph

    def simulate(self):
        for _ in range(self.substeps):
            self.state_0.clear_forces()
            self.state_1.clear_forces()
            self.integrator.simulate(self.model, self.state_0, self.state_1, self.dt)
            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        if self.use_cuda_graph:
            wp.capture_launch(self.graph)
        else:
            self.simulate()
        self.time += self.frame_dt

    def render(self):
        self.renderer.begin_frame(self.time)
        self.renderer.render(self.state_0)
        self.renderer.end_frame()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--stage_path", type=str, default="slideprinter.usd", help="USD output path"
    )
    parser.add_argument(
        "--num_frames", type=int, default=240, help="Number of frames to simulate"
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Override Warp device (defaults to cuda:0)",
    )
    args = parser.parse_args()

    # Default to CUDA device 0 for best performance
    with wp.ScopedDevice(args.device or "cuda:0"):
        example = SlideprinterExample(stage_path=args.stage_path)
        for _ in range(args.num_frames):
            example.step()
            example.render()
        example.renderer.save()
