import pytest
from unittest.mock import patch, AsyncMock
from move_commander import MoveCommander
import json
import numpy as np

@pytest.fixture
def gcode_file(tmp_path):
    return tmp_path / "test_movements.gcode"

@patch('move_commander.MoveCommander._get_dt', return_value=0.01)
def test_parse_gcode(mock_get_dt, gcode_file):
    with open(gcode_file, 'w') as f:
        f.write("G1 X1 Y1 Z3 F500\n")
        f.write("G1 X2 Y2 Z2 F1000\n")
    commander = MoveCommander(str(gcode_file), "ws://localhost:8766")
    assert len(commander.commands) == 2
    assert commander.commands[0] == {'type': 'G1', 'X': 1.0, 'Y': 1.0, 'Z': 3.0, 'speed': 500.0}
    assert commander.commands[1] == {'type': 'G1', 'X': 2.0, 'Y': 2.0, 'Z': 2.0, 'speed': 1000.0}

@pytest.mark.asyncio
@patch('websockets.connect')
@patch('asyncio.sleep', new_callable=AsyncMock)
@patch('move_commander.MoveCommander._get_dt', return_value=0.1)
@patch('move_commander.pos_to_motor_pos_samples_deg')
async def test_send_commands(mock_pos_to_motor, mock_get_dt, mock_sleep, mock_connect, gcode_file):
    def mock_kinematics(anchors, pos, low_axis_max_force, use_flex, spool_buildup_factor):
        # Mock kinematics where angle is proportional to the square of the x-coordinate.
        # This helps test that we are not just linearly interpolating angles.
        final_angle_rad = 8.33333281207e-06
        x = pos[0, 0]
        angle_rad = final_angle_rad * x**2
        angle_deg = angle_rad * 180.0 / np.pi
        # Return shape is (num_samples, num_motors)
        return np.array([[angle_deg, 0, 0]])

    mock_pos_to_motor.side_effect = mock_kinematics

    mock_websocket = AsyncMock()
    mock_connect.return_value.__aenter__.return_value = mock_websocket

    with open(gcode_file, 'w') as f:
        f.write("G1 X1 F60\n")

    commander = MoveCommander(str(gcode_file), 'ws://dummy')
    commander.dt = 0.1

    await commander.send_commands()

    assert mock_websocket.send.call_count == 10
    first_call = json.loads(mock_websocket.send.call_args_list[0].args[0])
    last_call = json.loads(mock_websocket.send.call_args_list[-1].args[0])

    # With our mock, angle_rad = final_angle_rad * x^2
    # First step is at t=0.1, so x=0.1. angle_rad = final_angle_rad * 0.1^2
    expected_first_angle = 8.33333281207e-06 * 0.01
    # Last step is at t=1.0, so x=1.0. angle_rad = final_angle_rad * 1^2
    expected_last_angle = 8.33333281207e-06

    assert first_call['action'] == 'gcode'
    assert pytest.approx(first_call['command']['A']) == expected_first_angle
    assert last_call['action'] == 'gcode'
    assert pytest.approx(last_call['command']['A']) == expected_last_angle
    assert pytest.approx(commander.current_angles_rad[0]) == expected_last_angle
