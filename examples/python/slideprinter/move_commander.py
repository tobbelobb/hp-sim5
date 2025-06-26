import asyncio
import websockets
import json
import re

class MoveCommander:
    def __init__(self, gcode_file, uri):
        self.gcode_file = gcode_file
        self.uri = uri
        self.commands = self._parse_gcode()

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
        async with websockets.connect(self.uri) as websocket:
            for command in self.commands:
                await websocket.send(json.dumps({'action': 'gcode', 'command': command}))
                # Wait for confirmation or just sleep
                await asyncio.sleep(1)

if __name__ == '__main__':
    commander = MoveCommander('movements.gcode', "ws://localhost:8766")
    asyncio.run(commander.send_commands())
