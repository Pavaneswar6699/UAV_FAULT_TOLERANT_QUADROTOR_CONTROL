"""Position MPC that runs on the laptop.

Model: three independent double integrators (x, y, z), input = world acceleration,
piecewise constant over each MPC step. The problem is a bounded linear least-squares
problem, solved with scipy.optimize.lsq_linear (no extra solver needed).

After the fault, the acceleration limits shrink to what the weakened motors can give,
so the plan stays inside what the drone can physically do.
"""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from .params import G


@dataclass
class MPCConfig:
    dt: float = 0.05        # s per MPC step
    horizon: int = 20       # steps (1 s)
    q_pos: float = 10.0
    q_vel: float = 1.0
    r_acc: float = 0.05


def acc_limits(p, fault_motor=0, thrust_cap=0.25, margin=0.8):
    """(lower, upper) world-acceleration bounds for x, y, z."""
    if fault_motor:
        # Weak motor and its diagonal partner are both limited to the cap (yaw dropped).
        # On a stock Crazyflie this is about the hover thrust or a little less, so the
        # MPC must never ask for a downward push: at worst the drone sinks slowly on its own.
        t_avail = (2 * thrust_cap + 2) * p.f_max
        az_hi = max(t_avail / p.mass - G, 0.0)
        az_lo = -1.0
        a_xy = 0.5
    else:
        az_hi = margin * (4 * p.f_max / p.mass - G)
        az_lo = -0.5 * G
        a_xy = 2.0
    return np.array([-a_xy, -a_xy, az_lo]), np.array([a_xy, a_xy, az_hi])


class PositionMPC:
    def __init__(self, cfg=MPCConfig()):
        self.cfg = cfg
        n, dt = cfg.horizon, cfg.dt
        k = np.arange(1, n + 1)[:, None]
        j = np.arange(n)[None, :]
        self.Mv = dt * (j < k)                                 # v_k from inputs
        self.Mp = dt * dt * np.where(j < k, k - j - 0.5, 0.0)  # p_k from inputs
        self.k = k[:, 0]

    def solve(self, p0, v0, ref, lo, hi):
        """ref: (horizon + 1, 3) reference positions starting at the current time.
        Returns (acc0, pos1, vel1): first acceleration and the predicted next state."""
        c = self.cfg
        n, dt = c.horizon, c.dt
        ref = np.asarray(ref)
        rv = np.diff(ref, axis=0) / dt
        acc, pos1, vel1 = np.zeros(3), np.zeros(3), np.zeros(3)
        for ax in range(3):
            p_free = p0[ax] + self.k * dt * v0[ax]
            v_free = np.full(n, v0[ax])
            A = np.vstack([np.sqrt(c.q_pos) * self.Mp,
                           np.sqrt(c.q_vel) * self.Mv,
                           np.sqrt(c.r_acc) * np.eye(n)])
            b = np.concatenate([np.sqrt(c.q_pos) * (ref[1:, ax] - p_free),
                                np.sqrt(c.q_vel) * (rv[:, ax] - v_free),
                                np.zeros(n)])
            lb, ub = lo[ax], max(hi[ax], lo[ax] + 1e-6)
            u = lsq_linear(A, b, bounds=(lb, ub), method="bvls").x
            acc[ax] = u[0]
            vel1[ax] = v0[ax] + dt * u[0]
            pos1[ax] = p0[ax] + dt * v0[ax] + 0.5 * dt * dt * u[0]
        return acc, pos1, vel1
