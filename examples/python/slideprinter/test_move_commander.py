import pytest
from unittest.mock import patch, AsyncMock
from move_commander import MoveCommander
import json

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
async def test_send_commands(mock_get_dt, mock_sleep, mock_connect, gcode_file):
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

    assert first_call['action'] == 'gcode'
    assert pytest.approx(first_call['command']['A']) == 8.33333281207e-07
    assert last_call['action'] == 'gcode'
    assert pytest.approx(last_call['command']['A']) == 8.33333281207e-06
    assert pytest.approx(commander.current_angles_rad[0]) == 8.33333281207e-06
