"""Linear Kalman filter that estimates wind from position and velocity (runs on the laptop, feeds MPC).

Per axis, with u the rotor thrust acceleration (known from the motor commands) and c the drag:
    p' = v
    v' = u - (c / m) v + d      d = wind acceleration, the unknown we want
    d' = noise                  wind changes slowly (random walk)
It measures p and v, which on the real drone come from the Crazyflie's onboard EKF.
Wind force [N] = mass * d; wind speed [m/s] = mass * d / drag.
"""
from typing import NamedTuple

import numpy as np

from quad import GRAVITY, QuadParams

# Tuning. Measurement noise should match the Loco/EKF output (bigger outdoors);
# the two process noises trade a smooth estimate against a fast one.
POS_STD = 0.01        # position measurement noise [m]
VEL_STD = 0.03        # velocity measurement noise [m/s]
ACC_PSD = 0.3         # thrust-model error [m/s^2 per sqrt(Hz)]
WIND_PSD = 0.3        # how fast the wind may change [m/s^2 per sqrt(s)]
WIND_STD_0 = 1.0      # initial wind uncertainty [m/s^2]

I3, Z3 = np.eye(3), np.zeros((3, 3))
H = np.hstack([np.eye(6), np.zeros((6, 3))])                     # we measure pos and vel
R = np.diag(np.repeat([POS_STD**2, VEL_STD**2], 3))


class WindEstimate(NamedTuple):
    x: np.ndarray     # [pos(3), vel(3), wind acceleration(3)]
    cov: np.ndarray   # 9 x 9

    @property
    def wind_acc(self) -> np.ndarray:
        return self.x[6:]


def initial_estimate(pos: np.ndarray, vel: np.ndarray) -> WindEstimate:
    x = np.concatenate([pos, vel, np.zeros(3)])
    return WindEstimate(x, np.diag(np.repeat([POS_STD**2, VEL_STD**2, WIND_STD_0**2], 3)))


def thrust_acceleration(rot: np.ndarray, thrust: np.ndarray, p: QuadParams) -> np.ndarray:
    """World-frame acceleration from the rotors and gravity (u in the model) [m/s^2]."""
    return rot[:, 2] * np.sum(thrust) / p.mass - np.array([0.0, 0.0, GRAVITY])


def predict(est: WindEstimate, u: np.ndarray, p: QuadParams, dt: float) -> WindEstimate:
    f = np.block([
        [I3, dt * I3, Z3],
        [Z3, (1.0 - p.lin_drag / p.mass * dt) * I3, dt * I3],
        [Z3, Z3, I3],
    ])
    x = f @ est.x + np.concatenate([np.zeros(3), u * dt, np.zeros(3)])
    q = np.diag(np.repeat([0.0, ACC_PSD**2 * dt, WIND_PSD**2 * dt], 3))
    return WindEstimate(x, f @ est.cov @ f.T + q)


def update(est: WindEstimate, pos: np.ndarray, vel: np.ndarray) -> WindEstimate:
    innovation = np.concatenate([pos, vel]) - H @ est.x
    s = H @ est.cov @ H.T + R
    gain = np.linalg.solve(s, H @ est.cov).T                      # K = P H^T S^-1
    i_kh = np.eye(9) - gain @ H
    cov = i_kh @ est.cov @ i_kh.T + gain @ R @ gain.T             # Joseph form: stays symmetric
    return WindEstimate(est.x + gain @ innovation, cov)
