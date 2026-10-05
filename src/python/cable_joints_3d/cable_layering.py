"""Stored-length to winding-angle mapping in the authored layer/ramp model."""
from dataclasses import dataclass
import math

EPSILON = 1e-9
MAX_LAYERS = 2048


@dataclass
class WindingState:
    radius: float
    theta: float
    layer: int
    phi: float
    in_ramp: bool


def _wrap_parameters(r0, dr, ramp_length, layer):
    radius = r0 + dr * layer
    ramp_angle = min(2 * math.pi, max(0., ramp_length / (radius + .5 * dr))) if ramp_length > EPSILON else 0.
    constant_angle = 2 * math.pi - ramp_angle
    constant_length = radius * constant_angle
    wrap_length = constant_length + ramp_angle * (radius + .5 * dr)
    return radius, ramp_angle, constant_angle, constant_length, wrap_length


def stored_to_radius_and_theta(stored_length, base_radius, half_width, ramp_length):
    stored = max(0., stored_length)
    r0, dr = base_radius + half_width, 2 * half_width
    if not (r0 > EPSILON and dr > EPSILON):
        radius = max(0., base_radius) if math.isfinite(base_radius) else 0.
        theta = stored / radius if radius > EPSILON else 0.
        return WindingState(radius, theta, 0, theta, False)
    remaining, theta_base = stored, 0.
    for layer in range(MAX_LAYERS):
        radius, ramp_angle, constant_angle, constant_length, wrap_length = _wrap_parameters(r0, dr, max(0., ramp_length), layer)
        if remaining > wrap_length + EPSILON:
            remaining -= wrap_length
            theta_base += 2 * math.pi
            continue
        if remaining <= constant_length + EPSILON or not ramp_angle > EPSILON:
            phi = min(constant_angle, remaining / radius) if radius > EPSILON else 0.
            return WindingState(radius, theta_base + phi, layer, phi, False)
        length_in_ramp = max(0., remaining - constant_length)
        a = dr / (2 * ramp_angle)
        x = (-radius + math.sqrt(max(0., radius * radius + 4 * a * length_in_ramp))) / (2 * a)
        x = min(ramp_angle, max(0., x))
        phi = constant_angle + x
        return WindingState(radius + dr * x / ramp_angle, theta_base + phi, layer, phi, True)
    return WindingState(r0 + dr * MAX_LAYERS, theta_base, MAX_LAYERS, 0., False)


def stored_to_theta_signed(stored_length, base_radius, half_width, ramp_length):
    if not math.isfinite(stored_length) or abs(stored_length) <= EPSILON:
        return 0.
    angle = stored_to_radius_and_theta(abs(stored_length), base_radius, half_width, ramp_length).theta
    return -angle if stored_length < 0 else angle
