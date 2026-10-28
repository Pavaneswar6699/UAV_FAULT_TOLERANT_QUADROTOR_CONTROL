"""Controllers: a stock-style cascade and our fault-tolerant INDI + WLS controller."""
from dataclasses import dataclass

import numpy as np

from allocation import FAULT_YAW_WEIGHT, NORMAL_WEIGHTS, stock_mix, wls_allocate
from quad import GRAVITY, QuadParams, allocation_matrix

KP_POS = 3.0                  # position error -> acceleration [1/s^2]
KD_POS = 2.5                  # velocity error -> acceleration [1/s]
K_ATT = 8.0                   # attitude error -> desired body rate [1/s]
K_RATE = 25.0                 # rate error -> desired angular acceleration [1/s]
NORMAL_TILT = np.deg2rad(25.0)
FAULT_TILT = np.deg2rad(15.0)
THRUST_MARGIN = 0.9
FAULT_CAP = 0.25              # 50% RPM leaves 25% of the thrust
YAW_RAMP_S = 0.2              # fade out heading control over this time
INDI_CUTOFF_HZ = 30.0


@dataclass(frozen=True)
class Sensors:
    pos: np.ndarray
    vel: np.ndarray
    rot: np.ndarray
    gyro: np.ndarray


@dataclass(frozen=True)
class Reference:
    pos: np.ndarray
    vel: np.ndarray
    yaw: float = 0.0


def thrust_vector(sens: Sensors, ref: Reference, p: QuadParams, max_tilt: float, thrust_max: float):
    """Position PD -> desired thrust direction (world frame) and collective thrust [N]."""
    a_cmd = KP_POS * (ref.pos - sens.pos) + KD_POS * (ref.vel - sens.vel)
    force = p.mass * (a_cmd + np.array([0.0, 0.0, GRAVITY]))
    fz = max(force[2], 0.3 * p.mass * GRAVITY)
    horiz = force[:2]
    limit = fz * np.tan(max_tilt)
    norm = np.linalg.norm(horiz)
    if norm > limit:
        horiz = horiz * (limit / norm)
    force = np.array([horiz[0], horiz[1], fz])
    n_des = force / np.linalg.norm(force)
    thrust = float(np.clip(force @ sens.rot[:, 2], 0.0, thrust_max))
    return n_des, thrust


def full_attitude_rates(rot: np.ndarray, n_des: np.ndarray, yaw_des: float) -> np.ndarray:
    """Normal mode: desired body rates to reach the thrust direction and the heading."""
    x_c = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0])
    y_b = np.cross(n_des, x_c)
    y_b = y_b / np.linalg.norm(y_b)
    r_des = np.column_stack([np.cross(y_b, n_des), y_b, n_des])
    e = r_des.T @ rot - rot.T @ r_des
    return -K_ATT * 0.5 * np.array([e[2, 1], e[0, 2], e[1, 0]])


def reduced_attitude_rates(rot: np.ndarray, n_des: np.ndarray) -> np.ndarray:
    """Fault mode: only steer the thrust direction (body z axis); heading is left free."""
    e = rot.T @ np.cross(rot[:, 2], n_des)
    return np.array([K_ATT * e[0], K_ATT * e[1], 0.0])


class StockController:
    """Stock-style cascade with a Crazyflie-style mixer. It is never told about the fault."""

    name = "stock"

    def __init__(self, p: QuadParams, dt: float):
        self.p = p

    def notify_fault(self, motor: int, t: float) -> None:
        """The stock controller has no fault interface."""

    def __call__(self, t: float, sens: Sensors, ref: Reference) -> np.ndarray:
        p = self.p
        n_des, thrust = thrust_vector(sens, ref, p, NORMAL_TILT, THRUST_MARGIN * 4 * p.f_max)
        w_des = full_attitude_rates(sens.rot, n_des, ref.yaw)
        inertia, w = p.inertia, sens.gyro
        tau = inertia @ (K_RATE * (w_des - w)) + np.cross(w, inertia @ w)
        return stock_mix(np.concatenate([[thrust], tau]), p)


class IndiWlsController:
    """Ours: position PD -> INDI on angular acceleration -> WLS allocation with per-motor limits.

    After the fault message it caps motor k at 25% thrust, releases heading (reduced attitude),
    and tightens the thrust and tilt limits.
    """

    name = "ours"

    def __init__(self, p: QuadParams, dt: float):
        self.p, self.dt = p, dt
        self.alpha = dt / (dt + 1.0 / (2.0 * np.pi * INDI_CUTOFF_HZ))
        hover = np.full(4, p.mass * GRAVITY / 4)
        self.f_est = hover
        self.w_f = np.zeros(3)
        self.tau_f = allocation_matrix(p)[1:] @ hover
        self.fault_motor = None
        self.fault_time = None

    def notify_fault(self, motor: int, t: float) -> None:
        self.fault_motor, self.fault_time = motor, t

    def _limits(self, t: float):
        p = self.p
        if self.fault_motor is None:
            return np.full(4, p.f_max), 0.0
        upper = np.where(np.arange(4) == self.fault_motor, FAULT_CAP * p.f_max, p.f_max)
        ramp = min(1.0, (t - self.fault_time) / YAW_RAMP_S)
        return upper, ramp

    def __call__(self, t: float, sens: Sensors, ref: Reference) -> np.ndarray:
        p, dt = self.p, self.dt
        upper, ramp = self._limits(t)
        faulted = self.fault_motor is not None
        # After the fault the weak diagonal pair is capped, so total thrust is about 2.5 motors' worth.
        thrust_max = THRUST_MARGIN * (2 * (1 + FAULT_CAP) if faulted else 4) * p.f_max
        n_des, thrust = thrust_vector(sens, ref, p, FAULT_TILT if faulted else NORMAL_TILT, thrust_max)
        w = sens.gyro
        w_des = reduced_attitude_rates(sens.rot, n_des) if faulted else full_attitude_rates(sens.rot, n_des, ref.yaw)
        nu = K_RATE * (w_des - w)

        # INDI: filter gyro and the motor-torque estimate the same way, then correct the difference.
        w_f = self.w_f + self.alpha * (w - self.w_f)
        w_dot_f = (w_f - self.w_f) / dt
        self.w_f = w_f
        self.tau_f = self.tau_f + self.alpha * (allocation_matrix(p)[1:] @ self.f_est - self.tau_f)
        tau_cmd = self.tau_f + p.inertia @ (nu - w_dot_f)

        yaw_weight = (1.0 - ramp) + ramp * FAULT_YAW_WEIGHT
        weights = NORMAL_WEIGHTS * np.array([1.0, 1.0, 1.0, yaw_weight])
        f_cmd = wls_allocate(np.concatenate([[thrust], tau_cmd]), p, upper, weights)
        # Motor model, clamped at the known cap so INDI does not credit motor k with thrust it can't give.
        self.f_est = self.f_est + (dt / p.motor_tau) * (np.clip(f_cmd, 0.0, upper) - self.f_est)
        return f_cmd
