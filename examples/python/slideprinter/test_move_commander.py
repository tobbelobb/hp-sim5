import unittest
from unittest.mock import Mock
from move_commander import MoveCommander

class TestMoveCommander(unittest.TestCase):
    def setUp(self):
        with open('test_movements.gcode', 'w') as f:
            f.write("G1 X1 Y1 Z3 F500\n")
            f.write("G1 X2 Y2 Z2 F1000\n")
        self.commander = MoveCommander('test_movements.gcode')

    def test_parse_gcode(self):
        self.assertEqual(len(self.commander.commands), 2)
        self.assertEqual(self.commander.commands[0], {'type': 'G1', 'A': 1.0, 'B': 1.0, 'C': 3.0, 'speed': 500.0})
        self.assertEqual(self.commander.commands[1], {'type': 'G1', 'A': 2.0, 'B': 2.0, 'C': 2.0, 'speed': 1000.0})

    def test_send_commands(self):
        mock_server = Mock()
        self.commander.send_commands(mock_server)
        self.assertEqual(mock_server.receive_command.call_count, 2)
        mock_server.receive_command.assert_any_call({'type': 'G1', 'A': 1.0, 'B': 1.0, 'C': 3.0, 'speed': 500.0})
        mock_server.receive_command.assert_any_call({'type': 'G1', 'A': 2.0, 'B': 2.0, 'C': 2.0, 'speed': 1000.0})

if __name__ == '__main__':
    unittest.main()
