import unittest
from unittest.mock import patch, AsyncMock
from move_commander import MoveCommander
import asyncio
import json

class TestMoveCommander(unittest.TestCase):
    def setUp(self):
        self.gcode_file = 'test_movements.gcode'

    @patch('move_commander.MoveCommander._get_dt', return_value=0.01)
    def test_parse_gcode(self, mock_get_dt):
        with open(self.gcode_file, 'w') as f:
            f.write("G1 X1 Y1 Z3 F500\n")
            f.write("G1 X2 Y2 Z2 F1000\n")
        commander = MoveCommander(self.gcode_file, "ws://localhost:8766")
        self.assertEqual(len(commander.commands), 2)
        self.assertEqual(commander.commands[0], {'type': 'G1', 'A': 1.0, 'B': 1.0, 'C': 3.0, 'speed': 500.0})
        self.assertEqual(commander.commands[1], {'type': 'G1', 'A': 2.0, 'B': 2.0, 'C': 2.0, 'speed': 1000.0})

    @patch('websockets.connect')
    @patch('asyncio.sleep', new_callable=AsyncMock)
    @patch('move_commander.MoveCommander._get_dt', return_value=0.1)
    async def test_send_commands(self, mock_get_dt, mock_sleep, mock_connect):
        mock_websocket = AsyncMock()
        mock_connect.return_value.__aenter__.return_value = mock_websocket

        # Use a simpler gcode for testing this specific behavior
        with open(self.gcode_file, 'w') as f:
            f.write("G1 A1 F60\n") # 1 rad/min -> 1/60 rad/sec

        commander = MoveCommander(self.gcode_file, 'ws://dummy')
        # Manually set dt for predictability, overriding the mock on the class
        commander.dt = 0.1
        
        await commander.send_commands()

        # Calculation:
        # distance = 1 rad.
        # speed = 60 rad/min = 1 rad/sec.
        # duration = distance / speed = 1s.
        # num_steps = ceil(duration / dt) = ceil(1s / 0.1s) = 10.
        self.assertEqual(mock_websocket.send.call_count, 10)

        # Check first call
        first_call_args = json.loads(mock_websocket.send.call_args_list[0].args[0])
        self.assertEqual(first_call_args['action'], 'gcode')
        self.assertAlmostEqual(first_call_args['command']['A'], 0.1) # 1.0 * (1/10)

        # Check last call
        last_call_args = json.loads(mock_websocket.send.call_args_list[-1].args[0])
        self.assertEqual(last_call_args['action'], 'gcode')
        self.assertAlmostEqual(last_call_args['command']['A'], 1.0) # 1.0 * (10/10)

        # Check that current angle is updated
        self.assertAlmostEqual(commander.current_angles['A'], 1.0)


if __name__ == '__main__':
    # To run async tests
    unittest.main()
