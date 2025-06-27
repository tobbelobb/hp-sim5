import asyncio
import websockets
import json
import re
from pathlib import Path
import math
import numpy as np

from .kinematics import pos_to_motor_pos_samples
from .guessed_data import guessed_anchors

class MoveCommander:
    def __init__(self, gcode_file, uri):
        self.gcode_file = gcode_file
        self.uri = uri
        self.commands = self._parse_gcode()
        self.dt = self._get_dt()
        self.current_angles_rad = np.array([0.0, 0.0, 0.0])
        self.anchors_mm = guessed_anchors
        self.current_pos_mm = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}

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

    async def send_commands(self):
        print(f"Connecting to {self.uri}")
        async with websockets.connect(self.uri) as websocket:
            print("Connection established. Sending commands...")
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

                    print(self.current_pos_mm)
                    print(target_pos_mm)
                    axesXYZ = ['X', 'Y', 'Z']
                    distance_mm = np.linalg.norm(
                        np.array([self.current_pos_mm[axis] for axis in axesXYZ]) -
                        np.array([target_pos_mm[axis] for axis in axesXYZ])
                    )
                    print(distance_mm)

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

                    print(f"Executing G1 move to {target_pos_mm} (mm) over {duration_s:.2f}s in {num_steps} steps.")

                    deltas_rad = target_angles_rad - self.current_angles_rad
                    axesABC = ['A', 'B', 'C']
                    for i in range(1, num_steps + 1):
                        t = i / num_steps
                        interpolated_cmd = {'type': 'G1'}
                        for idx, axis in enumerate(axesABC):
                            interpolated_cmd[axis] = self.current_angles_rad[idx] + deltas_rad[idx] * t

                        await websocket.send(json.dumps({'action': 'gcode', 'command': interpolated_cmd}))
                        await asyncio.sleep(self.dt)

                    self.current_angles_rad = target_angles_rad
                    self.current_pos_mm = target_pos_mm

            print("All commands sent.")

if __name__ == '__main__':
    from pathlib import Path
    root_dir = Path(__file__).resolve().parents[3]

    commander = MoveCommander(root_dir / "examples" / "python" / "slideprinter" / "test_movements.gcode", "ws://localhost:8766")
    asyncio.run(commander.send_commands())
