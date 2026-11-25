"""6-DOF Crazyflie simulator with the 50% RPM fault, plus the closed loop
(mission -> MPC -> onboard controller -> motors) used by scripts/run_sim.py."""
import numpy as np

from .allocation import mixer_matrix
from .controller import FTController
from .mission import Mission
from .mpc import MPCConfig, PositionMPC, acc_limits
from .params import G, Gains, QuadParams


def _rodrigues(w, dt):
    th = np.linalg.norm(w) * dt
    if th < 1e-12:
        return np.eye(3)
    k = w / np.linalg.norm(w)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


class QuadSim:
    def __init__(self, p, pos, dt=0.001, gyro_noise=0.02, seed=0):
        self.p, self.dt = p, dt
        self.pos, self.vel = np.asarray(pos, float), np.zeros(3)
        self.R, self.w = np.eye(3), np.zeros(3)
        self.f = np.full(4, p.weight / 4)
        self.B = mixer_matrix(p)
        self.inj_motor, self.inj_cap = 0, 0.25   # external limit (thrust fraction)
        self.rng = np.random.default_rng(seed)
        self.gyro_noise = gyro_noise

    def gyro(self):
        return self.w + self.gyro_noise * self.rng.standard_normal(3)

    def step(self, f_cmd):
        p, dt = self.p, self.dt
        f_cmd = np.clip(f_cmd, 0.0, p.f_max)
        if self.inj_motor:
            k = self.inj_motor - 1
            f_cmd[k] = min(f_cmd[k], self.inj_cap * p.f_max)
        self.f += (dt / p.tau_motor) * (f_cmd - self.f)
        wrench = self.B @ self.f
        tau = wrench[1:].copy()
        tau[2] -= p.yaw_drag * self.w[2] * abs(self.w[2])
        tau[:2] -= 2e-7 * self.w[:2]
        J = p.inertia
        acc = self.R @ np.array([0, 0, wrench[0]]) / p.mass - np.array([0, 0, G]) - p.lin_drag * self.vel / p.mass
        w_dot = np.linalg.solve(J, tau - np.cross(self.w, J @ self.w))
        self.vel += acc * dt
        self.pos += self.vel * dt
        self.w += w_dot * dt
        self.R = self.R @ _rodrigues(self.w, dt)
        u, _, vt = np.linalg.svd(self.R)
        self.R = u @ vt

    def tilt_deg(self):
        return np.degrees(np.arccos(np.clip(self.R[2, 2], -1, 1)))


def simulate(mode="ftc", fault_motor=1, fault_waypoint=2, notify_delay=0.0,
             p=None, g=None, mpc_cfg=None, t_max=40.0, seed=0):
    """Run one flight. mode: "ftc" (ours) or "pid" (baseline, never told about the fault).
    fault_motor=0 runs a healthy mission. Returns (log dict of arrays, summary dict)."""
    p = p or QuadParams()
    g = g or Gains()
    mpc_cfg = mpc_cfg or MPCConfig()
    start = np.array([0.0, 0.0, 0.0])
    mission = Mission(start)
    sim = QuadSim(p, pos=[0.0, 0.0, 0.0], seed=seed)
    sim.pos[2] = 0.6                      # start hovering at the take-off point
    mission_t0 = mission.waypoint_times[0]
    ctrl = FTController(p, g, mode)
    mpc = PositionMPC(mpc_cfg)
    t_fault = (mission.waypoint_times[fault_waypoint] - mission_t0 + 0.5) if fault_motor else None
    notified = False
    lo, hi = acc_limits(p)
    sp = (sim.pos.copy(), np.zeros(3), np.zeros(3))
    log = {k: [] for k in ["t", "x", "y", "z", "rx", "ry", "rz", "tilt", "yaw_rate",
                           "f1", "f2", "f3", "f4", "phase"]}
    summary = {"mode": mode, "fault_motor": fault_motor, "result": "timeout"}
    n_ctrl = int(round(g.dt / sim.dt))
    n_mpc = int(round(mpc_cfg.dt / sim.dt))
    n_log = 10
    phase, ref_now = "mission", sim.pos.copy()
    max_tilt_after = 0.0
    f_cmd = sim.f.copy()
    for i in range(int(t_max / sim.dt)):
        t = i * sim.dt
        tm = t + mission_t0
        if t_fault is not None and t >= t_fault and not sim.inj_motor:
            sim.inj_motor = fault_motor
        if t_fault is not None and t >= t_fault + notify_delay and not notified and mode == "ftc":
            notified = True
            ctrl.notify_fault(fault_motor, 0.25)
            mission.notify_fault(tm, sim.pos)
            lo, hi = acc_limits(p, fault_motor, 0.25)
        if i % n_mpc == 0:
            ref, phase = mission.ref_traj(tm, mpc_cfg.horizon, mpc_cfg.dt)
            ref_now = ref[0]
            acc, p1, v1 = mpc.solve(sim.pos, sim.vel, ref, lo, hi)
            sp = (p1, v1, acc)
        if i % n_ctrl == 0:
            f_cmd = ctrl.step(sim.pos, sim.vel, sim.R, sim.gyro(), sp[0], sp[1], sp[2], 0.0)
        sim.step(f_cmd)
        if sim.inj_motor:
            max_tilt_after = max(max_tilt_after, sim.tilt_deg())
        if i % n_log == 0:
            for k, val in zip(log.keys(), [t, *sim.pos, *ref_now, sim.tilt_deg(), sim.w[2],
                                           *sim.f, phase]):
                log[k].append(val)
        if sim.tilt_deg() > 100:
            summary.update(result="flipped", t_end=t)
            break
        if sim.pos[2] <= 0.0 and t > 1.0:
            ok = sim.tilt_deg() < 30 and sim.vel[2] > -0.8
            summary.update(result="landed" if ok else "crashed", t_end=t,
                           touchdown_speed=float(-sim.vel[2]), touchdown_tilt=float(sim.tilt_deg()))
            break
    summary.update(t_fault=t_fault, max_tilt_after_fault=float(max_tilt_after),
                   max_yaw_rate=float(np.max(np.abs(log["yaw_rate"]))) if log["yaw_rate"] else 0.0,
                   final_pos=[float(v) for v in sim.pos])
    return {k: np.array(v) for k, v in log.items()}, summary
