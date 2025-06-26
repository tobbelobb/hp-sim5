import asyncio
import websockets
import json
import re
from pathlib import Path
import math
import numpy as np

class MoveCommander:
    def __init__(self, gcode_file, uri):
        self.gcode_file = gcode_file
        self.uri = uri
        self.commands = self._parse_gcode()
        self.dt = self._get_dt()
        self.current_angles = {'A': 0.0, 'B': 0.0, 'C': 0.0}

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
                command['A'] = float(part[1:])
            elif part.startswith('Y'):
                command['B'] = float(part[1:])
            elif part.startswith('Z'):
                command['C'] = float(part[1:])
            elif part.startswith('F'):
                command['speed'] = float(part[1:])
        return command

    async def send_commands(self):
        print(f"Connecting to {self.uri}")
        async with websockets.connect(self.uri) as websocket:
            print("Connection established. Sending commands...")
            for command in self.commands:
                if command['type'] == 'G1':
                    target_angles = {}
                    for axis in ['A', 'B', 'C']:
                        if axis in command:
                            target_angles[axis] = command[axis]
                    
                    if not target_angles:
                        continue

                    deltas = {axis: target_angles.get(axis, self.current_angles[axis]) - self.current_angles[axis] for axis in ['A', 'B', 'C']}
                    distance = np.linalg.norm([deltas[axis] for axis in target_angles.keys()])
                    
                    # G-code speed is in units/minute. Assume radians.
                    speed_rad_per_min = command.get('speed', 60.0)
                    speed_rad_per_sec = speed_rad_per_min / 60.0

                    if distance < 1e-6 or speed_rad_per_sec < 1e-6:
                        continue

                    duration = distance / speed_rad_per_sec
                    num_steps = math.ceil(duration / self.dt)

                    if num_steps == 0:
                        continue
                    
                    print(f"Executing G1 move: {target_angles} over {duration:.2f}s in {num_steps} steps.")

                    for i in range(1, num_steps + 1):
                        t = i / num_steps
                        interpolated_cmd = {'type': 'G1'}
                        for axis, target_val in target_angles.items():
                            interpolated_cmd[axis] = self.current_angles[axis] + deltas[axis] * t
                        
                        await websocket.send(json.dumps({'action': 'gcode', 'command': interpolated_cmd}))
                        await asyncio.sleep(self.dt)

                    for axis, target_val in target_angles.items():
                        self.current_angles[axis] = target_val
            
            print("All commands sent.")

if __name__ == '__main__':
    commander = MoveCommander('test_movements.gcode', "ws://localhost:8766")
    asyncio.run(commander.send_commands())
