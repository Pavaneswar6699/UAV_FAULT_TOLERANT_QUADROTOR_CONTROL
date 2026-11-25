"""Motor mixing for the Crazyflie X layout, healthy and fault-aware.

Motor layout and signs match crazyflie-firmware power_distribution_quadrotor.c
(body frame x forward, y left, z up):

    M4 (front-left)   M1 (front-right)
    M3 (back-left)    M2 (back-right)

    T  =      f1 + f2 + f3 + f4
    tx = a * (-f1 - f2 + f3 + f4)          a = arm / sqrt(2)
    ty = a * (-f1 + f2 + f3 - f4)
    tz = k * (-f1 + f2 - f3 + f4)          k = THRUST2TORQUE

Diagonal pairs (M1, M3) and (M2, M4) spin the same way. The yaw pattern
n = [-1, 1, -1, 1] is orthogonal to the T, tx, ty rows, so adding lam * n to any
solution changes only the yaw torque. "Dropping yaw" means picking lam from the
range that keeps every motor inside its limits; with one weak motor this makes
its diagonal partner slow down to match it.
"""
import numpy as np

YAW_DIR = np.array([-1.0, 1.0, -1.0, 1.0])
ATTITUDE_SCALES = (1.0, 0.75, 0.5, 0.25, 0.0)   # how far roll/pitch torque is shrunk if nothing fits
FEASIBILITY_TOL = 1e-12                          # N, rounding slack in the feasibility tests


def mixer_matrix(p):
    a = p.arm / np.sqrt(2.0)
    k = p.k_torque
    return np.array([
        [1.0, 1.0, 1.0, 1.0],
        [-a, -a, a, a],
        [-a, a, a, -a],
        [-k, k, -k, k],
    ])


def base_solution(T, tx, ty, p):
    """Minimum-norm forces for T, tx, ty with zero yaw torque."""
    a = p.arm / np.sqrt(2.0)
    return 0.25 * (T + np.array([-tx - ty, -tx + ty, tx + ty, tx - ty]) / a)


def stock_mixer(v, p):
    """Emulates the stock firmware: invert, clamp at 0, cap by shifting all motors down."""
    f = np.linalg.solve(mixer_matrix(p), v)
    f = np.maximum(f, 0.0)
    excess = f.max() - p.f_max
    if excess > 0:
        f = np.maximum(f - excess, 0.0)
    return f


def lam_interval(f0, upper):
    """Range of lam such that 0 <= f0 + lam * YAW_DIR <= upper for every motor."""
    lo, hi = -np.inf, np.inf
    for fi, ui, ni in zip(f0, upper, YAW_DIR):
        a, b = (0.0 - fi) / ni, (ui - fi) / ni
        lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
    return lo, hi


def thrust_interval(tx: float, ty: float, upper: np.ndarray, p) -> tuple[float, float] | None:
    """Exact range [lo, hi] of collective thrust that fits inside the motor limits for these
    roll/pitch torques, with yaw left free. None if no thrust fits.

    With f = f0(T) + lam * YAW_DIR and f0 = T/4 + d, each motor gives a lower and an upper bound on
    lam that is linear in T. A lam exists iff every lower bound <= every upper bound: 16 linear
    inequalities in T, whose solution is an interval.
    """
    d = base_solution(0.0, tx, ty, p)
    spins_up = YAW_DIR > 0
    low0, low_slope = np.where(spins_up, -d, d - upper), np.where(spins_up, -0.25, 0.25)
    up0, up_slope = np.where(spins_up, upper - d, d), np.where(spins_up, -0.25, 0.25)
    const = (low0[:, None] - up0[None, :]).ravel()      # need const + slope * T <= 0
    slope = (low_slope[:, None] - up_slope[None, :]).ravel()
    if np.any(const[slope == 0.0] > FEASIBILITY_TOL):
        return None
    hi = float(np.min(-const[slope > 0] / slope[slope > 0], initial=np.inf))
    lo = float(np.max(-const[slope < 0] / slope[slope < 0], initial=-np.inf))
    if lo > hi + FEASIBILITY_TOL:
        return None
    return lo, max(hi, lo)


def fault_aware_mixer(v: np.ndarray, p, upper: np.ndarray) -> tuple[np.ndarray, dict]:
    """Allocate [T, tx, ty, tz] within per-motor upper limits.

    Priority: roll/pitch torque > thrust > yaw torque.
      1. Use the requested yaw torque if it fits.
      2. Otherwise drop yaw: pick lam in the feasible range, closest to the request.
      3. If no lam fits, move thrust to the nearest feasible value (exact, see thrust_interval).
      4. If still nothing fits, shrink the roll/pitch torque.
    Returns (forces, info).
    """
    T, tx, ty, tz = v
    upper = np.asarray(upper, dtype=float)
    lam_yaw = tz / (4.0 * p.k_torque)
    for att_scale in ATTITUDE_SCALES:
        interval = thrust_interval(att_scale * tx, att_scale * ty, upper, p)
        if interval is None:
            continue
        thrust = float(np.clip(T, *interval))
        f0 = base_solution(thrust, att_scale * tx, att_scale * ty, p)
        lo, hi = lam_interval(f0, upper)
        lam = min(max(lam_yaw, lo), hi)
        f = np.clip(f0 + lam * YAW_DIR, 0.0, upper)
        return f, {"yaw_dropped": not (lo <= lam_yaw <= hi), "thrust": thrust,
                   "att_scale": att_scale, "lam": lam}
    return np.zeros(4), {"yaw_dropped": True, "thrust": 0.0, "att_scale": 0.0, "lam": 0.0}
