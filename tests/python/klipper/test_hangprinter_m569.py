import importlib.util
from pathlib import Path


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
    def __init__(self, gcode):
        self.gcode = gcode

    def lookup_object(self, name):
        assert name == "gcode"
        return self.gcode


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer


class FakeGcmd:
    sentinel = object()

    def __init__(self, params):
        self.params = params
        self.responses = []

    def get(self, name, default=sentinel):
        if name in self.params:
            return self.params[name]
        return default

    def respond_raw(self, message):
        self.responses.append(message)


def build_handler():
    module = load_module()
    gcode = FakeGcode()
    config = FakeConfig(FakePrinter(gcode))
    module.load_config(config)
    return gcode.commands["M569.3"]


def test_registers_m569_3_command():
    command = build_handler()
    assert callable(command["func"])
    assert "placeholder" in command["desc"].lower()


def test_m569_3_requires_p_parameter():
    command = build_handler()
    gcmd = FakeGcmd({})
    command["func"](gcmd)
    assert gcmd.responses == ["Error: M569: missing parameter 'P'"]


def test_m569_3_emits_placeholder_message_when_p_is_present():
    command = build_handler()
    gcmd = FakeGcmd({"P": "40.0"})
    command["func"](gcmd)
    assert gcmd.responses == ["Error: M569.3: Message not received"]
