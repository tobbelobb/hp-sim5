import asyncio
import websockets
import json
import re
from pathlib import Path
import math
import numpy as np

import sys
from pathlib import Path

root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))

examples_python_path = root_dir / "examples" / "python"
if str(examples_python_path) not in sys.path:
    sys.path.insert(0, str(examples_python_path))

from slideprinter.forward_kinematics import slideprinter_forward_transform
from slideprinter.kinematics import pos_to_motor_pos_samples_deg, spool_r_in_origin_first_guess
from slideprinter.guessed_data import guessed_anchors

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
        self.last_speed_mm_per_min = 1000.0
        self.commands = self._parse_gcode()
        self.dt = self._get_dt()
        self.current_angles_rad = np.array([0.0, 0.0, 0.0])
        self.anchors_mm = guessed_anchors
        self.current_pos_mm = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.spool_radius_mm = spool_r_in_origin_first_guess
        self.low_axis_max_force = 20.0
        self.use_flex = False
        self.spool_buildup_factor = 0.0

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
            elif part.startswith('E'):
                command['E'] = float(part[1:])
            elif part.startswith('F'):
                self.last_speed_mm_per_min = float(part[1:])
        command['speed'] = self.last_speed_mm_per_min
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
                axesABC = ['A', 'B', 'C']
                axesXYZ = ['X', 'Y', 'Z']
                for command in self.commands:
                    if command['type'] == 'G1':
                        target_pos_mm = self.current_pos_mm.copy()
                        has_move = False
                        for axis in ['X', 'Y', 'Z']:
                            if axis in command:
                                # G-code is in mm, kinematics uses meters
                                target_pos_mm[axis] = command[axis]
                                has_move = True

                        extrusion_delta_mm = command.get('E', 0.0)

                        if not has_move and extrusion_delta_mm == 0.0:
                            continue

                        start_pos_mm_arr = np.array([self.current_pos_mm[axis] for axis in axesXYZ])
                        target_pos_mm_arr = np.array([target_pos_mm[axis] for axis in axesXYZ])
                        distance_mm = np.linalg.norm(target_pos_mm_arr - start_pos_mm_arr)

                        # G-code speed is in mm/min. Convert to m/s.
                        speed_mm_per_min = command['speed']
                        speed_mm_per_s = speed_mm_per_min / 60.0

                        duration_s = 0
                        if distance_mm > 1e-6 and speed_mm_per_s > 1e-6:
                            duration_s = distance_mm / speed_mm_per_s
                        
                        num_steps = 0
                        if duration_s > 0:
                            num_steps = math.ceil(duration_s / self.dt)

                        if num_steps > 0:
                            print(f"Executing G1 move to {target_pos_mm} (mm) over {duration_s:.2f}s in {num_steps} time steps.")

                            extrusion_per_step = extrusion_delta_mm / num_steps
                            final_angles_rad = None

                            for i in range(1, num_steps + 1):
                                t = i / num_steps
                                interp_pos_mm_arr = start_pos_mm_arr + (target_pos_mm_arr - start_pos_mm_arr) * t
                                pos_mm = np.array([interp_pos_mm_arr])
                                motor_positions_deg = pos_to_motor_pos_samples_deg(
                                    self.anchors_mm,
                                    pos_mm,
                                    self.low_axis_max_force,
                                    self.use_flex,
                                    spool_buildup_factor=self.spool_buildup_factor,
                                )
                                interp_angles_rad = motor_positions_deg[0]*(np.pi/180.0)
                                final_angles_rad = interp_angles_rad

                                interpolated_cmd = {'type': 'Move'}
                                for idx, axis in enumerate(axesABC):
                                    interpolated_cmd[axis] = interp_angles_rad[idx]

                                if extrusion_per_step > 0.0:
                                    interpolated_cmd['E'] = extrusion_per_step

                                await websocket.send(json.dumps({'action': 'gcode', 'command': interpolated_cmd}))
                                await asyncio.sleep(self.dt)

                            if final_angles_rad is not None:
                                self.current_angles_rad = final_angles_rad
                        else: # num_steps == 0
                            # A single step move, or just extrusion.
                            if has_move:
                                pos_mm = np.array([[target_pos_mm['X'], target_pos_mm['Y'], target_pos_mm['Z']]])
                                motor_positions_deg = pos_to_motor_pos_samples_deg(
                                    self.anchors_mm,
                                    pos_mm,
                                    self.low_axis_max_force,
                                    self.use_flex,
                                    spool_buildup_factor=self.spool_buildup_factor,
                                )
                                target_angles_rad = motor_positions_deg[0]*(np.pi/180.0)
                                self.current_angles_rad = target_angles_rad
                            else:
                                target_angles_rad = self.current_angles_rad

                            cmd = {'type': 'Move'}
                            for idx, axis in enumerate(axesABC):
                                cmd[axis] = target_angles_rad[idx]

                            if extrusion_delta_mm > 0.0:
                                cmd['E'] = extrusion_delta_mm
                            
                            if has_move or extrusion_delta_mm > 0.0:
                                await websocket.send(json.dumps({'action': 'gcode', 'command': cmd}))

                        self.current_pos_mm = target_pos_mm
                    elif command['type'] == 'G92':
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

                        print(f"Executing G92: setting current position to {target_pos_mm}")

                        target_angles_rad = pos_to_motor_pos_samples_deg(
                            self.anchors_mm,
                            pos_mm,
                            self.low_axis_max_force,
                            self.use_flex,
                            spool_buildup_factor=self.spool_buildup_factor,
                        )[0]*(np.pi/180.0)

                        deltas_rad = target_angles_rad.copy() - self.current_angles_rad.copy()
                        self.current_angles_rad = target_angles_rad
                        self.current_pos_mm = target_pos_mm
                        cmd = {'type': 'Add to reference'}

                        for idx, axis in enumerate(axesABC):
                            cmd[axis] = deltas_rad[idx]

                        await websocket.send(json.dumps({
                            'action': 'gcode',
                            'command': cmd
                        }))

                    elif command['type'] == 'G6':
                        line_deltas_mm = np.array([command.get(ax, 0.0) for ax in axesABC])

                        delta_angles_rad = line_deltas_mm / self.spool_radius_mm
                        target_angles_rad = self.current_angles_rad + delta_angles_rad

                        line_distance_mm = np.linalg.norm(line_deltas_mm)

                        # G-code speed is in mm/min. Convert to mm/s.
                        speed_mm_per_min = command.get('speed', 1000.0)  # default to 1000 mm/min
                        speed_mm_per_s = speed_mm_per_min / 60.0

                        duration_s = line_distance_mm / speed_mm_per_s
                        num_steps = math.ceil(duration_s / self.dt)

                        print(f"Executing G6 move with deltas {line_deltas_mm} (mm) over {duration_s:.2f}s in {num_steps} time steps.")

                        deltas_rad = target_angles_rad - self.current_angles_rad
                        for i in range(1, num_steps + 1):
                            t = i / num_steps
                            interpolated_cmd = {'type': 'Move'}
                            for idx, axis in enumerate(axesABC):
                                interpolated_cmd[axis] = self.current_angles_rad[idx] + deltas_rad[idx] * t

                            await websocket.send(json.dumps({'action': 'gcode', 'command': interpolated_cmd}))
                            await asyncio.sleep(self.dt)

                        cmd = {'type': 'Add to reference'}
                        for idx, axis in enumerate(axesABC):
                            cmd[axis] = deltas_rad[idx]

                        await websocket.send(json.dumps({
                            'action': 'gcode',
                            'command': cmd
                        }))

                print("All commands sent.")
                await asyncio.sleep(0.1) # Allow server to process and respond
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
