"""Simple Warp-based pendulum example.

See `ai_docs/Warp/README.md` for installation instructions and links to the
official Warp documentation.
"""

import math
import warp as wp
import warp.sim
import warp.sim.render

class PendulumExample:
    def __init__(self, stage_path="pendulum.usd"):
        self.frame_dt = 1.0 / 60.0
        self.substeps = 10
        self.dt = self.frame_dt / self.substeps
        self.time = 0.0

        builder = wp.sim.ModelBuilder()

        # Anchor particle at the origin (mass=0 makes it static)
        builder.add_particle(wp.vec3(0.0, 0.0, 0.0), wp.vec3(0.0, 0.0, 0.0), 0.0)

        # Pendulum bob one unit below the anchor
        length = 1.0
        mass = 1.0
        builder.add_particle(
            wp.vec3(0.0, -length, 0.0),
            wp.vec3(0.0, 0.0, 0.0),
            mass,
            radius=0.05,
        )

        # Distance spring keeps the bob at a fixed length from the anchor
        builder.add_spring(0, 1, ke=1.0e4, kd=0.0, control=0.0)

        self.model = builder.finalize()
        self.model.ground = False

        self.integrator = wp.sim.XPBDIntegrator()
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()

        self.renderer = None
        if stage_path:
            self.renderer = wp.sim.render.SimRenderer(self.model, stage_path, scaling=10.0)

        self.use_cuda_graph = wp.get_device().is_cuda and wp.is_mempool_enabled(wp.get_device())
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
        if self.renderer is None:
            return
        self.renderer.begin_frame(self.time)
        self.renderer.render(self.state_0)
        self.renderer.end_frame()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--stage_path", type=lambda x: None if x == "None" else str(x), default="pendulum.usd", help="USD output path")
    parser.add_argument("--num_frames", type=int, default=240, help="Number of frames to simulate")
    parser.add_argument("--device", type=str, default=None, help="Override Warp device")
    args = parser.parse_args()

    with wp.ScopedDevice(args.device):
        example = PendulumExample(stage_path=args.stage_path)
        for _ in range(args.num_frames):
            example.step()
            example.render()
        if example.renderer:
            example.renderer.save()
