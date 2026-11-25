"""Onboard controller (Python twin of firmware/app_ftc/src/ftc_core.c).

position tracking -> thrust vector -> yaw-independent tilt control -> rate loop -> mixer
"""
import numpy as np

from .allocation import fault_aware_mixer, mixer_matrix, stock_mixer
from .params import G


class Biquad:
    """2nd-order Butterworth low-pass (bilinear transform), applied elementwise."""

    def __init__(self, cutoff_hz, dt, n):
        w = np.tan(np.pi * cutoff_hz * dt)
        k = 1.0 / (1.0 + np.sqrt(2.0) * w + w * w)
        self.b = np.array([w * w * k, 2 * w * w * k, w * w * k])
        self.a = np.array([2 * (w * w - 1) * k, (1 - np.sqrt(2.0) * w + w * w) * k])
        self.x1 = self.x2 = self.y1 = self.y2 = np.zeros(n)
        self.primed = False

    def __call__(self, x):
        x = np.asarray(x, dtype=float)
        if not self.primed:
            self.x1 = self.x2 = self.y1 = self.y2 = x.copy()
            self.primed = True
        y = (self.b[0] * x + self.b[1] * self.x1 + self.b[2] * self.x2
             - self.a[0] * self.y1 - self.a[1] * self.y2)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


def yaw_of(R):
    return np.arctan2(R[1, 0], R[0, 0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class FTController:
    """mode "ftc": INDI rate loop + fault-aware mixing (our controller).
    mode "pid": plain rate loop + stock mixer, never told about the fault (baseline)."""

    def __init__(self, p, g, mode="ftc"):
        self.p, self.g, self.mode = p, g, mode
        self.B = mixer_matrix(p)
        self.gyro_lpf = Biquad(g.lpf_hz, g.dt, 3)
        self.tau_lpf = Biquad(g.lpf_hz, g.dt, 3)
        self.w_f_prev = None
        self.f_est = np.full(4, p.weight / 4)
        self.fault_motor = 0      # 1..4 once notified, 0 = healthy
        self.fault_cap = 0.25     # thrust fraction left on the weak motor (0.5 RPM -> 0.25 thrust)
        self.last = {}

    def notify_fault(self, motor, thrust_cap=0.25):
        if self.mode == "ftc":
            self.fault_motor, self.fault_cap = motor, thrust_cap

    def step(self, pos, vel, R, gyro, sp_pos, sp_vel, sp_acc, sp_yaw):
        p, g = self.p, self.g
        faulted = self.fault_motor > 0
        # 1. desired acceleration -> thrust vector (world frame)
        kp = np.array([g.kp_xy, g.kp_xy, g.kp_z])
        kd = np.array([g.kd_xy, g.kd_xy, g.kd_z])
        a = sp_acc + kp * (sp_pos - pos) + kd * (sp_vel - vel)
        a_xy_max = max(a[2] + G, 0.0) * np.tan(g.tilt_max)
        n_xy = np.linalg.norm(a[:2])
        if n_xy > a_xy_max:
            a[:2] *= a_xy_max / max(n_xy, 1e-9)
        t_vec = p.mass * (a + np.array([0.0, 0.0, G]))
        t_vec[2] = max(t_vec[2], 0.2 * p.weight)          # never ask to point the thrust down
        zb = R[:, 2]
        T = max(float(t_vec @ zb), 0.0)
        zd = t_vec / np.linalg.norm(t_vec)
        # 2. yaw-independent tilt error -> body-rate command
        e_b = R.T @ np.cross(zb, zd)
        w_des = np.array([g.k_att * e_b[0], g.k_att * e_b[1],
                          0.0 if faulted else g.k_yaw * wrap(sp_yaw - yaw_of(R))])
        # 3. rate loop
        J = np.diag(p.inertia)
        if self.mode == "ftc":
            w_f = self.gyro_lpf(gyro)
            dw_f = np.zeros(3) if self.w_f_prev is None else (w_f - self.w_f_prev) / g.dt
            self.w_f_prev = w_f
            tau_f = self.tau_lpf(self.B[1:] @ self.f_est)
            nu = g.k_rate * (w_des - w_f)
            tau = tau_f + J * (nu - dw_f)          # INDI: increment on the filtered actual torque
        else:
            tau = J * (g.k_rate * (w_des - gyro)) + np.cross(gyro, J * gyro)
        if faulted:
            tau[2] = 0.0                            # no yaw torque wanted; the mixer drops yaw
            # Spin compensation: the motors need ~tau_motor to respond, and meanwhile the body
            # turns by r * lead. Pre-rotate the roll/pitch torque so it lands where it is needed.
            psi = -gyro[2] * g.lead_time
            c, s = np.cos(psi), np.sin(psi)
            tau[0], tau[1] = c * tau[0] - s * tau[1], s * tau[0] + c * tau[1]
        v = np.array([T, tau[0], tau[1], tau[2]])
        # 4. mixing
        if self.mode == "ftc":
            upper = np.full(4, p.f_max)
            if faulted:
                upper[self.fault_motor - 1] = self.fault_cap * p.f_max
            f, info = fault_aware_mixer(v, p, upper)
        else:
            f, info = stock_mixer(v, p), {"yaw_dropped": False}
        self.f_est += (g.dt / p.tau_motor) * (f - self.f_est)
        self.last = {"T": T, "tau": tau, "info": info}
        return f
