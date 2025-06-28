import asyncio
import websockets
import json
import re
from pathlib import Path
import math
import numpy as np

from .forward_kinematics import slideprinter_forward_transform
from .kinematics import pos_to_motor_pos_samples, spool_r_in_origin_first_guess
from .guessed_data import guessed_anchors

async def _recv_and_discard(websocket):
    """Drain messages from the websocket connection to prevent it from blocking."""
    try:
        async for _ in websocket:
            pass
    except websockets.exceptions.ConnectionClosed:
        # The connection was closed, which is expected when the sender is done.
        pass

class MoveCommander:
    def __init__(self, gcode_file, uri):
        self.gcode_file = gcode_file
        self.uri = uri
        self.commands = self._parse_gcode()
        self.dt = self._get_dt()
        self.current_angles_rad = np.array([0.0, 0.0, 0.0])
        self.anchors_mm = guessed_anchors
        self.current_pos_mm = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.spool_radius_mm = spool_r_in_origin_first_guess

    def _get_dt(self):
        try:
            script_path = Path(__file__).resolve()
            root_dir = script_path.parents[3]
            scene_path = root_dir / "examples" / "usd_scenes" / "slideprinter.usda"
            with open(scene_path, 'r') as f:
                text = f.read()
            match = re.search(r"timeCodesPerSecond\s*=\s*(\d+(\.\d+)?)", text)
            if match:
                time_codes_per_second = float(match.group(1))
                print(f"Found timeCodesPerSecond: {time_codes_per_second}")
                return 1.0 / time_codes_per_second
        except Exception as e:
            print(f"Could not read dt from usd file: {e}")

        print("Falling back to dt=1/200s")
        return 1.0 / 200.0

    def _parse_gcode(self):
        commands = []
        with open(self.gcode_file, 'r') as f:
            for line in f:
                if line.startswith('G1'):
                    command = self._parse_g1_command(line)
                    if command:
                        commands.append(command)
                elif line.startswith('G6'):
                    command = self._parse_g6_command(line)
                    if command:
                        commands.append(command)
                elif line.startswith('G92'):
                    command = self._parse_g92_command(line)
                    if command:
                        commands.append(command)
        return commands

    def _parse_g1_command(self, line):
        parts = line.strip().split()
        command = {'type': 'G1'}
        for part in parts:
            if part.startswith('X'):
                command['X'] = float(part[1:])
            elif part.startswith('Y'):
                command['Y'] = float(part[1:])
            elif part.startswith('Z'):
                command['Z'] = float(part[1:])
            elif part.startswith('F'):
                command['speed'] = float(part[1:])
        return command

    def _parse_g92_command(self, line):
        parts = line.strip().split()
        command = {'type': 'G92'}
        has_coords = False
        for part in parts:
            if part.startswith('X'):
                command['X'] = float(part[1:])
                has_coords = True
            elif part.startswith('Y'):
                command['Y'] = float(part[1:])
                has_coords = True
            elif part.startswith('Z'):
                command['Z'] = float(part[1:])
                has_coords = True
        if has_coords:
            return command
        return None

    def _parse_g6_command(self, line):
        parts = line.strip().split()
        command = {'type': 'G6'}
        for part in parts:
            if part.startswith('A'):
                command['A'] = float(part[1:])
            elif part.startswith('B'):
                command['B'] = float(part[1:])
            elif part.startswith('C'):
                command['C'] = float(part[1:])
            elif part.startswith('F'):
                command['speed'] = float(part[1:])
        return command

    async def send_commands(self):
        print(f"Connecting to {self.uri}")
        async with websockets.connect(self.uri) as websocket:
            print("Connection established. Sending commands...")

            recv_task = asyncio.create_task(_recv_and_discard(websocket))

            try:
                for command in self.commands:
                    if command['type'] == 'G1':
                        target_pos_mm = self.current_pos_mm.copy()
                        has_move = False
                        for axis in ['X', 'Y', 'Z']:
                            if axis in command:
                                # G-code is in mm, kinematics uses meters
                                target_pos_mm[axis] = command[axis]
                                has_move = True

                        if not has_move:
                            continue

                        pos_mm = np.array([[target_pos_mm['X'], target_pos_mm['Y'], target_pos_mm['Z']]])
                        low_axis_max_force = 20.0
                        use_flex = False
                        spool_buildup_factor = 0.0

                        motor_positions_deg = pos_to_motor_pos_samples(
                            self.anchors_mm,
                            pos_mm,
                            low_axis_max_force,
                            use_flex,
                            spool_buildup_factor=spool_buildup_factor,
                        )

                        target_angles_rad = motor_positions_deg[0]*(np.pi/180.0)

                        axesXYZ = ['X', 'Y', 'Z']
                        distance_mm = np.linalg.norm(
                            np.array([self.current_pos_mm[axis] for axis in axesXYZ]) -
                            np.array([target_pos_mm[axis] for axis in axesXYZ])
                        )

                        # G-code speed is in mm/min. Convert to m/s.
                        speed_mm_per_min = command.get('speed', 1000.0)  # default to 1000 mm/min
                        speed_mm_per_s = speed_mm_per_min / 60.0

                        if distance_mm < 1e-6 or speed_mm_per_s < 1e-6:
                            self.current_pos_mm = target_pos_mm
                            self.current_angles_rad = target_angles_rad
                            continue

                        duration_s = distance_mm / speed_mm_per_s
                        num_steps = math.ceil(duration_s / self.dt)

                        if num_steps == 0:
                            self.current_pos_mm = target_pos_mm
                            self.current_angles_rad = target_angles_rad
                            continue

                        print(f"Executing G1 move to {target_pos_mm} (mm) over {duration_s:.2f}s in {num_steps} time steps.")

                        deltas_rad = target_angles_rad - self.current_angles_rad
                        axesABC = ['A', 'B', 'C']
                        for i in range(1, num_steps + 1):
                            t = i / num_steps
                            interpolated_cmd = {'type': 'Move'}
                            for idx, axis in enumerate(axesABC):
                                interpolated_cmd[axis] = self.current_angles_rad[idx] + deltas_rad[idx] * t

                            await websocket.send(json.dumps({'action': 'gcode', 'command': interpolated_cmd}))
                            await asyncio.sleep(self.dt)

                        self.current_angles_rad = target_angles_rad
                        self.current_pos_mm = target_pos_mm
                    elif command['type'] == 'G92':
                        p_set_mm = np.array([
                            command.get('X', self.current_pos_mm['X']),
                            command.get('Y', self.current_pos_mm['Y']),
                            command.get('Z', self.current_pos_mm['Z'])
                        ])

                        print(f"Executing G92: setting current position to {p_set_mm}")

                        # Based on angle * r = |pos - anchor| - |anchor|
                        anchor_norms = np.linalg.norm(self.anchors_mm, axis=1)
                        line_lengths_mm = self.current_angles_rad * self.spool_radius_mm + anchor_norms

                        p_phys_mm, spread = slideprinter_forward_transform(self.anchors_mm, line_lengths_mm)

                        if p_phys_mm is None:
                            print("G92 failed: forward kinematics could not find a solution.")
                            continue

                        print(f"G92: current physical position calculated as {p_phys_mm} with spread {spread}")

                        offset_mm = p_phys_mm - p_set_mm
                        self.anchors_mm = self.anchors_mm - offset_mm

                        self.current_pos_mm = {'X': p_set_mm[0], 'Y': p_set_mm[1], 'Z': p_set_mm[2]}

                        # Recalculate angles for new reference frame
                        new_anchor_norms = np.linalg.norm(self.anchors_mm, axis=1)
                        p_set_vec = np.array(list(self.current_pos_mm.values()))
                        new_line_lengths_to_pos = np.linalg.norm(p_set_vec - self.anchors_mm, axis=1)
                        self.current_angles_rad = (new_line_lengths_to_pos - new_anchor_norms) / self.spool_radius_mm

                        print(f"G92: new anchors at {self.anchors_mm.tolist()}")
                        print(f"G92: new current angles (rad) {self.current_angles_rad.tolist()}")

                        await websocket.send(json.dumps({
                            'action': 'configure',
                            'anchors': self.anchors_mm.tolist()
                        }))
                    elif command['type'] == 'G6':
                        axesABC = ['A', 'B', 'C']
                        line_deltas_mm = np.array([command.get(ax, 0.0) for ax in axesABC])

                        delta_angles_rad = line_deltas_mm / self.spool_radius_mm
                        target_angles_rad = self.current_angles_rad + delta_angles_rad

                        line_distance_mm = np.linalg.norm(line_deltas_mm)

                        # G-code speed is in mm/min. Convert to mm/s.
                        speed_mm_per_min = command.get('speed', 1000.0)  # default to 1000 mm/min
                        speed_mm_per_s = speed_mm_per_min / 60.0

                        if line_distance_mm < 1e-6 or speed_mm_per_s < 1e-6:
                            self.current_angles_rad = target_angles_rad
                            continue

                        duration_s = line_distance_mm / speed_mm_per_s
                        num_steps = math.ceil(duration_s / self.dt)

                        if num_steps == 0:
                            self.current_angles_rad = target_angles_rad
                            continue

                        print(f"Executing G6 move with deltas {line_deltas_mm} (mm) over {duration_s:.2f}s in {num_steps} time steps.")

                        deltas_rad = target_angles_rad - self.current_angles_rad
                        for i in range(1, num_steps + 1):
                            t = i / num_steps
                            interpolated_cmd = {'type': 'Move'}
                            for idx, axis in enumerate(axesABC):
                                interpolated_cmd[axis] = self.current_angles_rad[idx] + deltas_rad[idx] * t

                            await websocket.send(json.dumps({'action': 'gcode', 'command': interpolated_cmd}))
                            await asyncio.sleep(self.dt)

                        self.current_angles_rad = target_angles_rad

                print("All commands sent.")
            finally:
                recv_task.cancel()
                # Wait for the task to acknowledge cancellation
                await asyncio.gather(recv_task, return_exceptions=True)

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Send G-code commands from a file to the simulation.')
    parser.add_argument('gcode_file', help='Path to the G-code file.')
    args = parser.parse_args()

    commander = MoveCommander(args.gcode_file, "ws://localhost:8766")
    asyncio.run(commander.send_commands())
