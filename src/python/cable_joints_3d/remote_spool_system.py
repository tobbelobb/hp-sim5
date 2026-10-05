"""Headless command playback: one command per timestep, then motor prediction."""
from collections import deque
import warnings

from .ecs import MachineTagComponent, RenderableComponent
from .extruder import ExtruderComponent
from .machine_runtime import object_keys, set_stepper_position_mode, set_stepper_torque_mode
from .spools import SpoolStateComponent, SpoolTagComponent
from .stepper_motor import StepperMotorComponent, finite_number


def _copy_command(command):
    return command.copy() if isinstance(command, dict) else {}


def _emit(listener, value):
    if listener is not None:
        try:
            listener(value)
        except Exception as error:
            warnings.warn(f'Command listener failed: {error}', stacklevel=2)


def _machine_color(colors, machine):
    record = colors.get(machine) if isinstance(colors, dict) else None
    color = record.get('extrusionColor') if isinstance(record, dict) else None
    return color if isinstance(color, str) and color else None


class RemoteSpoolSystem:
    def __init__(self):
        self._commands = deque()
        self.axis_to_entity = {}
        self.history = []
        self.on_command_executed = None
        self.on_extrusion = None

    @property
    def commands(self):
        return self._commands

    @commands.setter
    def commands(self, values):
        self._commands = deque(values) if isinstance(values, (list, tuple, deque)) else deque()

    def add_command(self, command):
        self._commands.append(command)

    def get_queue_length(self):
        return len(self._commands)

    def clear_command_queue(self):
        self._commands.clear()

    def clear_playback_state(self):
        self.history = []
        self.clear_command_queue()

    def get_playback_state(self):
        return {'history': [_copy_command(command) for command in self.history],
                'queue': [_copy_command(command) for command in self._commands]}

    def set_playback_state(self, state):
        history, queue = state.get('history'), state.get('queue')
        self.history = [_copy_command(command) for command in history] if isinstance(history, list) else []
        self._commands = deque(_copy_command(command) for command in queue) if isinstance(queue, list) else deque()

    def reset_axis_mapping(self):
        self.axis_to_entity = {}

    def set_command_executed_listener(self, listener):
        self.on_command_executed = listener if callable(listener) else None

    def set_extrusion_listener(self, listener):
        self.on_extrusion = listener if callable(listener) else None

    def _ensure_axis_mapping(self, world):
        if self.axis_to_entity:
            return
        for entity in world.query([SpoolTagComponent, SpoolStateComponent]):
            state = world.get_component(entity, SpoolStateComponent)
            if state.axis:
                self.axis_to_entity.setdefault(str(state.axis), []).append(entity)

    def process_command(self, world, command, *, record_history=True, emit_events=True):
        if command is None:
            return
        recorded = _copy_command(command)
        if record_history:
            self.history.append(recorded)
        if emit_events:
            _emit(self.on_command_executed, recorded)
        command_type = command.get('type') or ''
        if command_type in ('SetTorqueMode', 'SetPositionMode'):
            mapping = self.axis_to_entity.get(command.get('axis'))
            entities = mapping if isinstance(mapping, list) else ([] if mapping is None else [mapping])
            for entity in entities:
                if command_type == 'SetTorqueMode':
                    set_stepper_torque_mode(world, entity, command.get('torqueNm') or 0.)
                else:
                    set_stepper_position_mode(world, entity)
            return
        touched, colors = {}, {}
        machine_colors = world.get_resource('machineColors')
        for axis in object_keys(self.axis_to_entity):
            value = command.get(axis)
            if value is None:
                axes = command.get('axes')
                value = axes.get(axis) if isinstance(axes, dict) else None
            if value is None:
                continue
            mapping = self.axis_to_entity[axis]
            if not mapping:
                continue
            for entity in mapping if isinstance(mapping, list) else [mapping]:
                tag = world.get_component(entity, MachineTagComponent)
                if tag is None or not tag.id:
                    continue
                machine = tag.id
                touched[machine] = None
                if machine not in colors:
                    color = _machine_color(machine_colors, machine)
                    render = world.get_component(entity, RenderableComponent)
                    color = color or (render.color if render else None)
                    if color:
                        colors[machine] = color
                stepper = world.get_component(entity, StepperMotorComponent)
                if stepper is not None:
                    if command_type == 'Move' and not stepper.torque_mode:
                        stepper.commanded_angle = value
                    elif command_type == 'Add to reference':
                        stepper.delta_angle += value
        if record_history and touched:
            recorded['__touchedMachines'] = list(touched)
        if command.get('E', 0) is None or command.get('E', 0) <= 0:
            return
        extruders = world.query([ExtruderComponent])
        if not extruders:
            return
        extruder = world.get_component(extruders[0], ExtruderComponent)
        machines = list(touched)
        if not machines and isinstance(command.get('__touchedMachines'), list):
            machines = command['__touchedMachines'][:]
        tip_keys, center_keys = object_keys(extruder.machine_tips), object_keys(extruder.machine_centers)
        if not machines and len(tip_keys) == 1:
            machines = tip_keys[:]
        elif not machines and len(center_keys) == 1:
            machines = center_keys[:]
        for machine in machines:
            if not machine:
                continue
            tip = extruder.machine_tips.get(machine)
            if tip is None:
                tip = extruder.machine_centers.get(machine)
            if tip is None:
                if not tip_keys and not center_keys and extruder.tip_pos is not None:
                    tip = extruder.tip_pos
                elif not center_keys and extruder.center_pos is not None:
                    tip = extruder.center_pos
                else:
                    continue
            event = {'pos': [float(tip[0]), float(tip[1]), float(tip[2]) if finite_number(tip[2]) else 0.],
                     'length': command['E'], 'machineId': machine,
                     'color': colors.get(machine) or _machine_color(machine_colors, machine)}
            extruder.extrusions.append(event)
            if emit_events:
                _emit(self.on_extrusion, event)

    def update(self, world, dt_unused):
        self._ensure_axis_mapping(world)
        if self._commands:
            self.process_command(world, self._commands.popleft())
