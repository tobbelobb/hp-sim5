"""1-DOF replica of the engine's closed-loop servo step (engine.rs motors/servo
+ XPBD line row with the spool's J+K dt^2 inertia and torque-limit release).

Spool (J, radius r) -> line (stiffness k) -> effector mass m, which also hangs
on a stiff 'other lines' spring k2 and carries a static load F so the motor
holds tau_L = F r in steady state. Returns tail amplitude of rotor error.
"""
import numpy as np


def run(T=1.0, J=5e-5, npp=50, dt=2e-3, r=0.03, k=13000.0, k2=40000.0, m=0.05,
        load=0.8, steps=6000, kick=0.0, servo=None, iters=4, release=True, mode='explicit'):
    F = load / r
    kp, kd, ki = (servo or default_gains)(T, npp, J, dt)
    th, w, x, v = 0.0, 0.0, 0.0, 0.0
    # Start at static equilibrium: line tension F, servo integrator = load
    x = -F / k2 * 0  # other spring takes nothing; line holds F
    stretch = F / k            # line length C = x - r*th + L0 ; tension k*stretch
    x = stretch                # effector pulled away from spool by `stretch`
    I = -load                  # servo holds -load (line pulls theta positive)
    e_prev = 0.0
    text = load  # line torque on the rotor
    th += kick
    errs = []
    for n in range(steps):
        # motors(): servo
        e = 0.0 - th
        cmd = kp * e + I + kd * (e - e_prev) / dt
        Kl = kp + kd / dt
        if mode == 'explicit':
            sat = abs(cmd) >= T
        elif mode == 'motor_only':
            sat = abs(cmd) * J / (J + Kl * dt * dt) >= T
        elif mode == 'with_load':
            # end-of-step servo torque if the line keeps last step's torque
            c = dt * dt / J
            uend = (cmd - Kl * c * text) / (1 + Kl * c)
            sat = abs(uend) > T
        elif mode == 'realized':
            sat = abs(cmd) / (1 + Kl * dt * dt / J) > T
        if not sat:
            I = np.clip(I + ki * e * dt, -T, T)
        e_prev = e
        K = 0.0 if sat else kp + kd / dt
        tau = np.clip(cmd, -T, T) if mode != 'realized' or sat else cmd
        if mode == 'realized' and sat: tau = np.sign(cmd) * T
        if mode == 'with_load': tau = np.sign(uend) * T if sat else cmd
        w += dt * tau / (J + K * dt * dt)
        # integrate
        th_s, x_s = th + dt * w, x + dt * v + dt * dt * (-F) / m * 0
        # external force on effector: F pulls effector toward +x? effector
        # load F acts on x in +x direction, line pulls back toward spool.
        x_s = x + dt * v + dt * dt * F / m
        # XPBD: row1 line C1 = x - r th (tension k*C1 >= 0), row2 other C2 = x
        Jeff = J + K * dt * dt
        lam1 = lam2 = 0.0
        a1, a2 = 1 / (k * dt * dt), 1 / (k2 * dt * dt)
        for _ in range(iters):
            ws = 1 / Jeff
            C1 = x_s - r * th_s
            if release and abs(lam1) / (dt * dt) * r > T:
                ws = 1 / J
            d = -(C1 + a1 * lam1) / (1 / m + r * r * ws + a1)
            nl = min(lam1 + d, 0.0); d = nl - lam1; lam1 = nl
            x_s += d / m
            th_s += -r * ws * d
            if release and abs(lam1) / (dt * dt) * r > T and ws != 1 / J:
                pass
            C2 = x_s
            d2 = -(C2 + a2 * lam2) / (1 / m + a2)
            nl = min(lam2 + d2, 0.0); d2 = nl - lam2; lam2 = nl
            x_s += d2 / m
        text = -lam1 / (dt * dt) * r
        w = (th_s - th) / dt
        v = (x_s - x) / dt
        th, x = th_s, x_s
        errs.append(e)
    errs = np.array(errs)
    tail = errs[-1000:]
    return tail.max() - tail.min(), errs


def default_gains(T, npp, J, dt):
    kp = T * npp
    return kp, 2 * np.sqrt(kp * J), kp / 1.0


if __name__ == "__main__":
    for T in [0.5, 0.9, 1.0, 1.2, 1.5, 2.0]:
        for kick in [0.0, 0.01, 0.05]:
            amp, _ = run(T=T, kick=kick)
            print(f"T={T} kick={kick}: tail p2p err {amp:.2e}")
