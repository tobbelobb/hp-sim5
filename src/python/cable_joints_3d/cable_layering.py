"""Signed winding-angle and stored-length mapping in the layer/ramp model."""
from dataclasses import dataclass
import math

EPSILON = 1e-9
MAX_LAYERS = 2048
KNOT_SPAN = math.pi / 30


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


def theta_to_stored_length(theta, base_radius, half_width, ramp_length):
    if not math.isfinite(theta) or abs(theta) <= EPSILON:
        return 0.
    if theta < 0:
        return -theta_to_stored_length(-theta, base_radius, half_width, ramp_length)
    r0, dr = base_radius + half_width, 2 * half_width
    if not (r0 > EPSILON and dr > EPSILON):
        return (max(0., base_radius) if math.isfinite(base_radius) else 0.) * theta
    remaining, stored, layer = theta, 0., 0
    while remaining > 2 * math.pi + EPSILON and layer < MAX_LAYERS:
        stored += _wrap_parameters(r0, dr, max(0., ramp_length), layer)[4]
        remaining -= 2 * math.pi
        layer += 1
    if layer >= MAX_LAYERS:
        return stored + (r0 + dr * MAX_LAYERS) * remaining
    radius, ramp_angle, constant_angle, constant_length, _ = _wrap_parameters(r0, dr, max(0., ramp_length), layer)
    phi = min(2 * math.pi, max(0., remaining))
    if not ramp_angle > EPSILON or phi <= constant_angle + EPSILON:
        return stored + radius * min(constant_angle, phi)
    x = phi - constant_angle
    return stored + constant_length + radius * x + dr * x * x / (2 * ramp_angle)


def hybrid_stored_delta(theta_before, delta_angle, cw, base_radius, half_width, radius_fallback):
    signed_delta = (1 if cw else -1) * delta_angle
    if not math.isfinite(signed_delta) or abs(signed_delta) <= EPSILON:
        return 0.
    if math.isfinite(base_radius) and math.isfinite(half_width) and half_width > EPSILON:
        ramp_length = max(0., base_radius) * KNOT_SPAN
        start = theta_before if math.isfinite(theta_before) else 0.
        return theta_to_stored_length(start + signed_delta, base_radius, half_width, ramp_length) - theta_to_stored_length(start, base_radius, half_width, ramp_length)
    radius = radius_fallback if math.isfinite(radius_fallback) else (base_radius if math.isfinite(base_radius) else 0.)
    return radius * signed_delta


def hybrid_angle_correction(theta_before, delta_angle, cw, base_radius, half_width, stored_shift, radius_fallback):
    if not math.isfinite(stored_shift) or abs(stored_shift) <= EPSILON:
        return 0.
    direction = 1 if cw else -1
    signed_delta = direction * delta_angle
    if not math.isfinite(signed_delta):
        return 0.
    if math.isfinite(base_radius) and math.isfinite(half_width) and half_width > EPSILON:
        ramp_length = max(0., base_radius) * KNOT_SPAN
        start = theta_before if math.isfinite(theta_before) else 0.
        after = theta_to_stored_length(start + signed_delta, base_radius, half_width, ramp_length)
        target = stored_to_theta_signed(after + stored_shift, base_radius, half_width, ramp_length)
        correction = (target - start) / direction - delta_angle
        return correction if math.isfinite(correction) else 0.
    radius = radius_fallback if math.isfinite(radius_fallback) else (base_radius if math.isfinite(base_radius) else 0.)
    return stored_shift / (direction * radius) if abs(radius) > EPSILON else 0.


def cable_stored_length_after_rotation(world, path, index, entity, delta_angle):
    from .ecs import RadiusComponent, layering_enabled

    radius = world.get_component(entity, RadiusComponent)
    base_radius = radius.radius if radius else 0.
    half_width = path.cable_half_width if layering_enabled(world) else 0.
    stored = path.stored[index]
    endpoint_sign = 1 if index == 0 else -1
    if path.link_types[index] != 'hybrid':
        return stored + endpoint_sign * (1 if path.cw[index] else -1) * delta_angle * (base_radius + half_width)
    theta = stored_to_theta_signed(stored, base_radius, half_width, base_radius * KNOT_SPAN)
    return stored + hybrid_stored_delta(theta, endpoint_sign * delta_angle, path.cw[index], base_radius, half_width, base_radius + half_width)
