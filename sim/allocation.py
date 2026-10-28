"""Motor allocation: a Crazyflie-style mixer and the fault-aware WLS allocator."""
import numpy as np
from scipy.optimize import lsq_linear

from quad import QuadParams, allocation_matrix

# Priorities for [thrust, roll, pitch, yaw]: stay level first, then hold height, then heading.
NORMAL_WEIGHTS = np.array([10.0, 100.0, 100.0, 1.0])
FAULT_YAW_WEIGHT = 1e-3
REGULARISATION = 1e-4


def stock_mix(wrench: np.ndarray, p: QuadParams) -> np.ndarray:
    """Invert B; if a motor is over its maximum, lower all four equally (as the Crazyflie firmware does).

    It assumes every motor has the same maximum, so it never knows about a weak motor.
    """
    f = np.linalg.solve(allocation_matrix(p), wrench)
    excess = f.max() - p.f_max
    if excess > 0.0:
        f = f - excess
    return np.clip(f, 0.0, p.f_max)


def wls_allocate(wrench: np.ndarray, p: QuadParams, f_upper: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """Motor thrusts closest to the wanted [T, tau], within each motor's own limit, by priority.

    Solves  min ||W (B f - wrench)||^2 + r ||f - f_pref||^2  subject to  0 <= f <= f_upper.
    Rows are scaled by each axis's full authority, so the weights express priority, not units.
    """
    b_mat = allocation_matrix(p)
    row_scale = np.abs(b_mat).sum(axis=1) * p.f_max
    w = np.asarray(weights) / row_scale
    a = (w[:, None] * b_mat) * p.f_max          # unknowns are f / f_max
    b = w * np.asarray(wrench)
    x_pref = np.full(4, wrench[0] / (4 * p.f_max))
    r = np.sqrt(REGULARISATION)
    solution = lsq_linear(
        np.vstack([a, r * np.eye(4)]),
        np.concatenate([b, r * x_pref]),
        bounds=(np.zeros(4), np.asarray(f_upper) / p.f_max),
        method="bvls",
    )
    return solution.x * p.f_max
