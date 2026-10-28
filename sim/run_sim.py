"""Fly the rectangle mission with a mid-path motor fault, using the stock controller and ours.

Examples (from the sim folder):
  .venv/bin/python run_sim.py                    # both controllers, M1 fault midway along B -> C
  .venv/bin/python run_sim.py --no-fault         # baseline flights only
  .venv/bin/python run_sim.py --yaw-drag 2e-6    # test sensitivity to an assumed parameter
"""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from control import FAULT_CAP, IndiWlsController, Reference, Sensors, StockController  # noqa: E402
from quad import GRAVITY, MOTOR_NAMES, QuadParams, hover_state, step  # noqa: E402

PHYSICS_DT = 0.001
CONTROL_EVERY = 2             # controller runs at 500 Hz
HEIGHT = 0.8
START_HOLD_S = 2.0
SETTLE_S = 2.5
CHECK_WINDOW_S = 1.0
MAX_POS_ERR = 0.3             # health check: position error [m]
MAX_TILT_DEG = 20.0           # health check: tilt from vertical [deg]
LAND_SPEED = 0.2
LANDED_Z = 0.03
CRASH_TILT_DEG = 80.0
GYRO_NOISE = 0.01
POS_NOISE = 0.003
VEL_NOISE = 0.01
RESULTS = Path(__file__).parent / "results"
MODES = ("hold", "fly", "settle", "continue", "land")


class Mission:
    """Rectangle A -> B -> C -> D -> A with the settle / health-check / decide logic after a fault."""

    def __init__(self, side: float, speed: float):
        self.corners = np.array([[0, 0], [side, 0], [side, side], [0, side], [0, 0]], float)
        self.side, self.speed = side, speed
        self.s = 0.0
        self.mode = "hold"
        self.mode_start = 0.0
        self.samples = []             # (t, position error, tilt) while settling
        self.decision = None
        self.land_xy = None
        self.land_z = HEIGHT

    def _point(self, s: float):
        seg = min(int(s // self.side), 3)
        a, b = self.corners[seg], self.corners[seg + 1]
        direction = (b - a) / self.side
        return a + direction * (s - seg * self.side), direction

    def _start_landing(self, xy):
        self.mode, self.land_xy = "land", np.asarray(xy, float)

    def fault(self, t: float) -> None:
        self.mode, self.mode_start = "settle", t

    def update(self, t: float, dt: float, pos: np.ndarray, tilt_deg: float) -> Reference:
        if self.mode == "hold" and t - self.mode_start >= START_HOLD_S:
            self.mode = "fly"
        if self.mode in ("fly", "continue"):
            speed = self.speed if self.mode == "fly" else self.speed / 2
            self.s = self.s + speed * dt
            if self.s >= 4 * self.side:
                self._start_landing(self.corners[0])
        xy, direction = self._point(min(self.s, 4 * self.side - 1e-9))
        if self.mode == "land":
            self.land_z = max(0.0, self.land_z - LAND_SPEED * dt)
            return Reference(np.array([*self.land_xy, self.land_z]), np.array([0, 0, -LAND_SPEED]))
        ref_pos = np.array([*xy, HEIGHT])
        if self.mode == "settle":
            self.samples.append((t, np.linalg.norm(pos - ref_pos), tilt_deg))
            if t - self.mode_start >= SETTLE_S:
                self._health_check(t, xy)
            return Reference(ref_pos, np.zeros(3))
        moving = self.mode in ("fly", "continue")
        speed = self.speed if self.mode == "fly" else self.speed / 2
        return Reference(ref_pos, np.array([*(direction * speed * moving), 0.0]))

    def _health_check(self, t: float, xy) -> None:
        recent = [(e, tilt) for (ts, e, tilt) in self.samples if ts >= t - CHECK_WINDOW_S]
        worst_err = max(e for e, _ in recent)
        worst_tilt = max(tilt for _, tilt in recent)
        passed = worst_err < MAX_POS_ERR and worst_tilt < MAX_TILT_DEG
        self.decision = (f"{'PASS' if passed else 'FAIL'} (pos err {worst_err:.2f} m, "
                         f"tilt {worst_tilt:.1f} deg)")
        if passed:
            self.mode = "continue"
        else:
            self._start_landing(xy)


def simulate(controller_cls, p: QuadParams, args, fault_time):
    rng = np.random.default_rng(1)
    state = hover_state(p, [0.0, 0.0, HEIGHT])
    mission = Mission(args.side, args.speed)
    ctrl = controller_cls(p, PHYSICS_DT * CONTROL_EVERY)
    motor = args.fault_motor - 1
    cap = np.full(4, p.f_max)
    f_cmd = state.thrust
    faulted = False
    log = []
    outcome = "time ran out"
    fault_pos = None
    for i in range(int(args.duration / PHYSICS_DT)):
        t = i * PHYSICS_DT
        if fault_time is not None and not faulted and t >= fault_time:
            cap = np.where(np.arange(4) == motor, FAULT_CAP * p.f_max, p.f_max)
            ctrl.notify_fault(motor, t)
            mission.fault(t)
            faulted, fault_pos = True, state.pos.copy()
        tilt = float(np.degrees(np.arccos(np.clip(state.rot[2, 2], -1.0, 1.0))))
        if i % CONTROL_EVERY == 0:
            sens = Sensors(
                pos=state.pos + rng.normal(0, POS_NOISE, 3),
                vel=state.vel + rng.normal(0, VEL_NOISE, 3),
                rot=state.rot,
                gyro=np.clip(state.omega + rng.normal(0, GYRO_NOISE, 3), -p.gyro_limit, p.gyro_limit),
            )
            ref = mission.update(t, PHYSICS_DT * CONTROL_EVERY, state.pos, tilt)
            f_cmd = ctrl(t, sens, ref)
            log.append([t, *state.pos, *ref.pos, tilt, state.omega[2], *state.thrust,
                        *state.rot.ravel(), MODES.index(mission.mode), float(faulted)])
        state = step(state, f_cmd, cap, p, PHYSICS_DT)
        if tilt > CRASH_TILT_DEG or (state.pos[2] < LANDED_Z and mission.mode != "land"):
            outcome = "CRASHED (flipped or hit the ground)"
            break
        if mission.mode == "land" and state.pos[2] < LANDED_Z:
            done = mission.s >= 4 * args.side
            outcome = "finished the rectangle and landed at A" if done else "landed safely before finishing"
            break
    return np.array(log), outcome, mission.decision, fault_pos


def summarise(name, log, outcome, decision, p):
    yaw_peak = np.degrees(np.abs(log[:, 8])).max()
    err = np.linalg.norm(log[:, 1:4] - log[:, 4:7], axis=1)
    print(f"[{name}] {outcome}")
    if decision:
        print(f"    health check: {decision}")
    print(f"    max position error {err.max():.2f} m · max tilt {log[:, 7].max():.1f} deg · "
          f"peak spin {yaw_peak:.0f} deg/s (gyro limit {p.gyro_limit_dps:.0f})")


def plot(results, args, fault_time, p):
    RESULTS.mkdir(exist_ok=True)
    colors = {"stock": "tab:gray", "ours": "tab:orange"}
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5))
    ax_xy, ax_z, ax_tilt, ax_yaw = axes.ravel()
    rect = np.array([[0, 0], [args.side, 0], [args.side, args.side], [0, args.side], [0, 0]])
    ax_xy.plot(rect[:, 0], rect[:, 1], "k--", lw=1, label="planned path")
    for name, (log, outcome, _, fault_pos) in results.items():
        c = colors[name]
        ax_xy.plot(log[:, 1], log[:, 2], color=c, lw=1.5, label=f"{name}: {outcome}")
        if fault_pos is not None:
            ax_xy.plot(*fault_pos[:2], "o", color="tab:red", ms=8)
        ax_z.plot(log[:, 0], log[:, 3], color=c, label=name)
        ax_tilt.plot(log[:, 0], log[:, 7], color=c, label=name)
        ax_yaw.plot(log[:, 0], np.degrees(log[:, 8]), color=c, label=name)
    for ax in (ax_z, ax_tilt, ax_yaw):
        if fault_time is not None:
            ax.axvline(fault_time, color="tab:red", lw=1, ls=":", label="fault")
        ax.set_xlabel("time [s]")
        ax.grid(alpha=0.3)
    ax_yaw.axhline(p.gyro_limit_dps, color="k", ls="--", lw=0.8, label="gyro limit")
    ax_yaw.axhline(-p.gyro_limit_dps, color="k", ls="--", lw=0.8)
    ax_xy.set(title="Path seen from above (red dot = fault)", xlabel="x [m]", ylabel="y [m]", aspect="equal")
    ax_z.set(title="Height", ylabel="z [m]")
    ax_tilt.set(title="Tilt from vertical", ylabel="deg")
    ax_yaw.set(title="Spin (yaw rate)", ylabel="deg/s")
    ax_xy.legend(fontsize=8, loc="upper left", bbox_to_anchor=(0, -0.12))
    ax_z.legend(fontsize=8)
    ax_xy.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / "compare.png", dpi=120)

    if "ours" in results:
        log = results["ours"][0]
        fig2, ax = plt.subplots(figsize=(10, 4))
        for m in range(4):
            ax.plot(log[:, 0], log[:, 9 + m] / GRAVITY * 1000, label=MOTOR_NAMES[m])
        ax.axhline(FAULT_CAP * p.f_max / GRAVITY * 1000, color="tab:red", ls="--", lw=1, label="weak motor cap")
        if fault_time is not None:
            ax.axvline(fault_time, color="tab:red", lw=1, ls=":")
        ax.set(title="Our controller: motor thrusts", xlabel="time [s]", ylabel="thrust [g]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, ncol=5)
        fig2.tight_layout()
        fig2.savefig(RESULTS / "motors_ours.png", dpi=120)
    for name, (log, *_rest) in results.items():
        np.savetxt(RESULTS / f"log_{name}.csv", log, delimiter=",", fmt="%.5f",
                   header="t,x,y,z,x_ref,y_ref,z_ref,tilt_deg,yaw_rate,f1,f2,f3,f4,r00,r01,r02,r10,r11,r12,r20,r21,r22,mode,faulted")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controller", choices=["both", "stock", "ours"], default="both")
    ap.add_argument("--fault-motor", type=int, default=1, choices=[1, 2, 3, 4])
    ap.add_argument("--fault-time", type=float, help="default: halfway along B -> C")
    ap.add_argument("--no-fault", action="store_true")
    ap.add_argument("--side", type=float, default=1.5, help="rectangle side [m]")
    ap.add_argument("--speed", type=float, default=0.3, help="flight speed [m/s]")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--yaw-drag", type=float, help="override assumed yaw damping")
    ap.add_argument("--motor-tau", type=float, help="override assumed motor time constant [s]")
    args = ap.parse_args()

    p = QuadParams()
    if args.yaw_drag is not None:
        p = replace(p, yaw_drag=args.yaw_drag)
    if args.motor_tau is not None:
        p = replace(p, motor_tau=args.motor_tau)
    fault_time = None if args.no_fault else (
        args.fault_time if args.fault_time is not None
        else START_HOLD_S + 1.5 * args.side / args.speed)

    controllers = {"stock": StockController, "ours": IndiWlsController}
    names = ["stock", "ours"] if args.controller == "both" else [args.controller]
    results = {}
    for name in names:
        log, outcome, decision, fault_pos = simulate(controllers[name], p, args, fault_time)
        results[name] = (log, outcome, decision, fault_pos)
        summarise(name, log, outcome, decision, p)
    plot(results, args, fault_time, p)
    meta = {
        "side": args.side, "fault_motor": args.fault_motor, "fault_time": fault_time, "modes": MODES,
        "hover_thrust": p.mass * GRAVITY / 4,
        "outcomes": {name: r[1] for name, r in results.items()},
    }
    (RESULTS / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"plots and logs saved in {RESULTS}")


if __name__ == "__main__":
    main()
