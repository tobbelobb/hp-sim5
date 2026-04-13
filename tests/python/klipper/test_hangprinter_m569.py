import importlib.util
import math
from pathlib import Path

import pytest


ROOT_DIR = Path(__file__).resolve().parents[3]
MODULE_PATH = ROOT_DIR / "klipper" / "klippy" / "extras" / "hangprinter_m569.py"


def load_module():
    spec = importlib.util.spec_from_file_location("hangprinter_m569", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeGcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, func, when_not_ready=False, desc=None):
        self.commands[name] = {
            "func": func,
            "when_not_ready": when_not_ready,
            "desc": desc,
        }


class FakePrinter:
    sentinel = object()

    def __init__(self, gcode, toolhead=None):
        self.gcode = gcode
        self.toolhead = toolhead

    def lookup_object(self, name, default=sentinel):
        if name == "gcode":
            return self.gcode
        if name == "toolhead" and self.toolhead is not None:
            return self.toolhead
        if default is not self.sentinel:
            return default
        raise AssertionError(f"Unexpected lookup_object({name!r})")


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer


class FakeStepper:
    def __init__(self, name, rotation_distance, invert_dir=False):
        self.name = name
        self.rotation_distance = rotation_distance
        self.invert_dir = invert_dir

    def get_rotation_distance(self):
        return self.rotation_distance, 3200

    def get_dir_inverted(self):
        return self.invert_dir, self.invert_dir

    def get_name(self):
        return self.name


class FakeFlexHelper:
    def __init__(self, mechanical_advantage):
        self.mechanical_advantage = list(mechanical_advantage)


class FakeKinematics:
    def __init__(self, steppers, mechanical_advantage, m569_driver_descriptors=None):
        self.steppers = list(steppers)
        self.flex_helper = FakeFlexHelper(mechanical_advantage)
        self.m569_driver_descriptors = list(m569_driver_descriptors or [])

    def get_steppers(self):
        return list(self.steppers)


class FakeToolhead:
    def __init__(self, kinematics):
        self.kinematics = kinematics

    def get_kinematics(self):
        return self.kinematics


class FakeGcmd:
    sentinel = object()

    def __init__(self, params, raw_command_parameters=None):
        self.params = params
        self.responses = []
        self.raw_command_parameters = raw_command_parameters

    def get(self, name, default=sentinel):
        if name in self.params:
            return self.params[name]
        return default

    def get_raw_command_parameters(self):
        return self.raw_command_parameters

    def respond_raw(self, message):
        self.responses.append(message)


def build_handler():
    module = load_module()
    gcode = FakeGcode()
    config = FakeConfig(FakePrinter(gcode))
    handler = module.load_config(config)
    return handler, gcode.commands["M569.3"]


def build_m569_with_runtime(
    steppers=None,
    mechanical_advantage=None,
    m569_driver_descriptors=None,
):
    module = load_module()
    gcode = FakeGcode()
    steppers = steppers or []
    mechanical_advantage = mechanical_advantage or []
    toolhead = FakeToolhead(
        FakeKinematics(steppers, mechanical_advantage, m569_driver_descriptors)
    )
    config = FakeConfig(FakePrinter(gcode, toolhead=toolhead))
    handler = module.load_config(config)
    return handler, gcode.commands


def test_registers_m569_3_command():
    _handler, command = build_handler()
    assert callable(command["func"])
    assert "placeholder" in command["desc"].lower()


def test_m569_3_requires_p_parameter():
    _handler, command = build_handler()
    gcmd = FakeGcmd({})
    command["func"](gcmd)
    assert gcmd.responses == ["Error: M569: missing parameter 'P'"]


def test_m569_3_emits_placeholder_message_when_p_is_present():
    _handler, commands = build_m569_with_runtime(
        steppers=[FakeStepper("stepper_a", 2.0 * math.pi * 39.1845)],
        mechanical_advantage=[1],
    )
    gcmd = FakeGcmd({"P": "stepper_a"})
    commands["M569.3"]["func"](gcmd)
    assert gcmd.responses == ["Error: M569.3: Message not received"]


def test_registers_m569_4_command():
    _handler, commands = build_m569_with_runtime()
    command = commands["M569.4"]
    assert callable(command["func"])
    assert "torque mode" in command["desc"].lower()


def test_m569_4_requires_p_parameter():
    _handler, commands = build_m569_with_runtime()
    gcmd = FakeGcmd({"T": "1.0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["Error: M569: missing parameter 'P'"]


def test_m569_4_requires_t_parameter():
    _handler, commands = build_m569_with_runtime()
    gcmd = FakeGcmd({"P": "40.0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["Error: M569.4 missing parameter 'T'"]


def test_m569_4_allows_single_t_value_for_all_p_entries():
    handler, commands = build_m569_with_runtime(
        steppers=[
            FakeStepper("stepper_a", 2.0 * math.pi * 39.1845, invert_dir=False),
            FakeStepper("stepper_b", 2.0 * math.pi * 39.1845, invert_dir=True),
        ],
        mechanical_advantage=[1, 1],
    )
    gcmd = FakeGcmd({"P": "stepper_a:stepper_b", "T": "1.0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["-0.039185 Nm, 0.039185 Nm, "]
    assert handler.driver_states["stepper_a"]["force_newtons"] == 1.0
    assert handler.driver_states["stepper_b"]["force_newtons"] == 1.0


def test_m569_4_requires_one_t_value_per_p_when_t_is_not_scalar():
    _handler, commands = build_m569_with_runtime(
        steppers=[
            FakeStepper("stepper_a", 2.0 * math.pi * 39.1845),
            FakeStepper("stepper_b", 2.0 * math.pi * 39.1845),
            FakeStepper("stepper_c", 2.0 * math.pi * 39.1845),
        ],
        mechanical_advantage=[1, 1, 1],
    )
    gcmd = FakeGcmd({"P": "stepper_a:stepper_b:stepper_c", "T": "1.0:2.0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["M569.4 requires one T value per P"]


def test_m569_4_converts_force_to_torque_and_tracks_placeholder_state():
    handler, commands = build_m569_with_runtime(
        steppers=[
            FakeStepper("stepper_a", 2.0 * math.pi * 39.1845, invert_dir=False),
            FakeStepper("stepper_b", 2.0 * math.pi * 39.1845, invert_dir=True),
        ],
        mechanical_advantage=[1, 1],
    )
    gcmd = FakeGcmd({"P": "stepper_a:stepper_b", "T": "1.0:2.0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["-0.039185 Nm, 0.078369 Nm, "]
    assert handler.driver_states["stepper_a"]["force_newtons"] == 1.0
    assert handler.driver_states["stepper_a"]["position_mode"] is False
    assert handler.driver_states["stepper_a"]["torque_nm"] == pytest.approx(-0.0391845)
    assert handler.driver_states["stepper_b"]["force_newtons"] == 2.0
    assert handler.driver_states["stepper_b"]["position_mode"] is False
    assert handler.driver_states["stepper_b"]["torque_nm"] == pytest.approx(0.078369)


def test_m569_4_switches_back_to_position_mode_below_threshold():
    handler, commands = build_m569_with_runtime(
        steppers=[FakeStepper("stepper_a", 2.0 * math.pi * 39.1845)],
        mechanical_advantage=[1],
    )
    gcmd = FakeGcmd({"P": "stepper_a", "T": "0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["pos_mode, "]
    assert handler.driver_states["stepper_a"] == {
        "force_newtons": 0.0,
        "torque_nm": 0.0,
        "position_mode": True,
    }


def test_m569_4_uses_configured_driver_descriptors():
    handler, commands = build_m569_with_runtime(
        steppers=[FakeStepper("stepper_a", 2.0 * math.pi * 39.1845)],
        mechanical_advantage=[1],
        m569_driver_descriptors=[{
            "stepper_name": "stepper_a",
            "motor_address": {"can_address": 55, "driver": 0},
        }],
    )
    gcmd = FakeGcmd({"P": "55.0", "T": "1.0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["-0.039185 Nm, "]
    assert handler.driver_states["55.0"]["force_newtons"] == 1.0


def test_m569_4_reports_missing_m569_address_for_named_stepper():
    _handler, commands = build_m569_with_runtime(
        steppers=[
            FakeStepper("stepper_a", 2.0 * math.pi * 39.1845),
            FakeStepper("stepper_b", 2.0 * math.pi * 39.1845),
        ],
        mechanical_advantage=[1, 1],
        m569_driver_descriptors=[
            {
                "stepper_name": "stepper_a",
                "motor_address": {"can_address": 55, "driver": 0},
            },
            {
                "stepper_name": "stepper_b",
                "motor_address": None,
            },
        ],
    )
    gcmd = FakeGcmd({"P": "stepper_a:stepper_b", "T": "1.0:2.0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["m569_address not configured for stepper_b"]


def test_m569_4_reports_unknown_tokens():
    _handler, commands = build_m569_with_runtime(
        steppers=[FakeStepper("stepper_a", 2.0 * math.pi * 39.1845)],
        mechanical_advantage=[1],
    )
    gcmd = FakeGcmd({"P": "does_not_exist", "T": "1.0"})
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["does_not_exist did not match any stepper name or m569_address."]


def test_m569_4_parses_raw_string_parameters():
    handler, commands = build_m569_with_runtime(
        steppers=[
            FakeStepper("stepper_a", 2.0 * math.pi * 39.1845, invert_dir=False),
            FakeStepper("stepper_b", 2.0 * math.pi * 39.1845, invert_dir=True),
        ],
        mechanical_advantage=[1, 1],
    )
    gcmd = FakeGcmd({}, raw_command_parameters="Pstepper_a:stepper_b T3:2.0")
    commands["M569.4"]["func"](gcmd)
    assert gcmd.responses == ["-0.117554 Nm, 0.078369 Nm, "]
    assert handler.driver_states["stepper_a"]["force_newtons"] == 3.0
    assert handler.driver_states["stepper_b"]["force_newtons"] == 2.0
