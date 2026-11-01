"""Crazyflie-sized quadrotor: parameters, motor layout and rigid-body dynamics.

Frames: world is x-east, y-north, z-up; body is x-forward, y-left, z-up.
Motor order follows the Crazyflie firmware: M1 front-right, M2 back-right,
M3 back-left, M4 front-left. Diagonal pairs (M1, M3) and (M2, M4) spin the same way.
"""
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

GRAVITY = 9.81
MOTOR_NAMES = ("M1 front-right", "M2 back-right", "M3 back-left", "M4 front-left")
NO_WIND = np.zeros(3)


@dataclass(frozen=True)
class QuadParams:
    # From gym-pybullet-drones cf2x.urdf (Crazyflie 2.x).
    mass: float = 0.027
    arm: float = 0.0397
    kf: float = 3.16e-10
    km: float = 7.94e-12
    ixx: float = 1.4e-5
    iyy: float = 1.4e-5
    izz: float = 2.17e-5
    thrust_to_weight: float = 2.25
    # Assumptions, not measured. Change them to see how sensitive the result is.
    # Defaults are a middle-of-range guess; the README lists the cases that fail.
    motor_tau: float = 0.03      # motor time constant [s]; measure on the bench
    lin_drag: float = 0.01       # translational drag [N per m/s]
    rot_drag_xy: float = 1e-6    # roll/pitch damping [N m s]
    yaw_drag: float = 2e-5       # quadratic yaw damping [N m s^2]; sets the spin rate after a fault (~700 deg/s)
    gyro_limit_dps: float = 2000.0

    @property
    def f_max(self) -> float:
        """Maximum thrust of one healthy motor [N]."""
        return self.thrust_to_weight * self.mass * GRAVITY / 4

    @property
    def kappa(self) -> float:
        """Drag torque per newton of thrust [m]."""
        return self.km / self.kf

    @property
    def inertia(self) -> np.ndarray:
        return np.diag([self.ixx, self.iyy, self.izz])

    @property
    def gyro_limit(self) -> float:
        return float(np.deg2rad(self.gyro_limit_dps))


class State(NamedTuple):
    pos: np.ndarray
    vel: np.ndarray
    rot: np.ndarray     # body-to-world rotation matrix
    omega: np.ndarray   # body rates [rad/s]
    thrust: np.ndarray  # actual motor thrusts [N]


def allocation_matrix(p: QuadParams) -> np.ndarray:
    """B in [T, tau_x, tau_y, tau_z] = B @ [f1, f2, f3, f4], same signs as the Crazyflie mixer."""
    d = p.arm / np.sqrt(2.0)
    k = p.kappa
    return np.array([
        [1.0, 1.0, 1.0, 1.0],
        [-d, -d, d, d],
        [-d, d, d, -d],
        [-k, k, -k, k],
    ])


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rotation_from_vector(phi: np.ndarray) -> np.ndarray:
    """Rotation matrix for a rotation vector (Rodrigues' formula)."""
    angle = np.linalg.norm(phi)
    if angle < 1e-12:
        return np.eye(3) + skew(phi)
    k = skew(phi / angle)
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * k @ k


def hover_state(p: QuadParams, pos) -> State:
    return State(np.asarray(pos, float), np.zeros(3), np.eye(3), np.zeros(3),
                 np.full(4, p.mass * GRAVITY / 4))


def step(state: State, f_cmd: np.ndarray, motor_cap: np.ndarray, p: QuadParams, dt: float,
         wind: np.ndarray = NO_WIND) -> State:
    """Advance the physics by dt. motor_cap is each motor's real maximum: the fault is applied here.
    wind is the air velocity [m/s]; drag acts on the velocity relative to the air."""
    target = np.clip(f_cmd, 0.0, motor_cap)
    thrust = state.thrust + (target - state.thrust) * (dt / p.motor_tau)
    wrench = allocation_matrix(p) @ thrust
    acc = (state.rot @ np.array([0.0, 0.0, wrench[0]]) - p.lin_drag * (state.vel - wind)) / p.mass
    acc = acc - np.array([0.0, 0.0, GRAVITY])
    w = state.omega
    drag = np.array([-p.rot_drag_xy * w[0], -p.rot_drag_xy * w[1], -p.yaw_drag * w[2] * abs(w[2])])
    inertia = p.inertia
    w_dot = np.linalg.solve(inertia, wrench[1:] + drag - np.cross(w, inertia @ w))
    vel = state.vel + acc * dt
    omega = w + w_dot * dt
    return State(
        pos=state.pos + vel * dt,
        vel=vel,
        rot=state.rot @ rotation_from_vector(omega * dt),
        omega=omega,
        thrust=thrust,
    )
