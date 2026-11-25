"""Fly the waypoint mission on the real Crazyflie with motion capture, MPC on the laptop.

    # healthy baseline with the stock PID controller (works on stock firmware)
    python scripts/hw_fly_mission.py --controller pid --no-fault
    # our controller, fault on motor 1 at waypoint 2 (we emulate the 50% limit ourselves)
    python scripts/hw_fly_mission.py --controller ftc --fault-motor 1 --fault-waypoint 2
    # demo day, if the evaluator applies the limit externally: only send the notification
    python scripts/hw_fly_mission.py --controller ftc --fault-motor 1 --fault-key --no-inject

Keys while flying:  f = trigger the fault now,  q (or Ctrl+C) = KILL motors.
Logs go to logs/flight_YYYYMMDD_HHMMSS.csv (plot with scripts/plot_log.py).
"""
import argparse
import csv
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cflib.crtp  # noqa: E402
from cflib.crazyflie import Crazyflie  # noqa: E402
from cflib.crazyflie.log import LogConfig  # noqa: E402
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  # noqa: E402

from ftqc.mission import DEFAULT_WAYPOINTS, Mission  # noqa: E402
from ftqc.mpc import MPCConfig, PositionMPC, acc_limits  # noqa: E402
from ftqc.params import QuadParams  # noqa: E402

CONTROLLER_ID = {"pid": "1", "indi": "3", "ftc": "6"}


class KeyReader:
    """Non-blocking single-key reader for Windows and Linux terminals."""

    def __init__(self):
        self.win = os.name == "nt"
        if not self.win:
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def get(self):
        if self.win:
            import msvcrt
            return msvcrt.getwch() if msvcrt.kbhit() else None
        import select
        return sys.stdin.read(1) if select.select([sys.stdin], [], [], 0)[0] else None

    def close(self):
        if not self.win:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class MocapBridge(threading.Thread):
    """Streams the rigid-body pose from the lab motion-capture system to the Crazyflie."""

    def __init__(self, cf, mocap_type, host, body):
        super().__init__(daemon=True)
        self.cf, self.mocap_type, self.host, self.body = cf, mocap_type, host, body
        self.running = True
        self.last_pos = None

    def run(self):
        import motioncapture
        mc = motioncapture.connect(self.mocap_type, {"hostname": self.host})
        while self.running:
            mc.waitForNextFrame()
            obj = mc.rigidBodies.get(self.body)
            if obj is None:
                continue
            p, q = obj.position, obj.rotation
            self.last_pos = np.array(p, dtype=float)
            self.cf.extpos.send_extpose(p[0], p[1], p[2], q.x, q.y, q.z, q.w)


def add_logs(cf, state, rows_lock, ftc):
    groups = [
        ("est", ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                 "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz"], "float"),
        ("att", ["stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw", "gyro.z"], "float"),
    ]
    if ftc:
        groups.append(("ftc", ["ftc.f1", "ftc.f2", "ftc.f3", "ftc.f4", "ftc.thrust"], "float"))
    configs = []
    for name, variables, typ in groups:
        lc = LogConfig(name=name, period_in_ms=20)
        for v in variables:
            lc.add_variable(v, typ)
        if name == "ftc":
            lc.add_variable("ftc.mode", "uint8_t")

        def cb(ts, data, logconf):
            with rows_lock:
                state.update(data)
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(cb)
        lc.start()
        configs.append(lc)
    return configs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uri", default="radio://0/80/2M/E7E7E7E7E7")
    ap.add_argument("--mocap-type", default="optitrack", help="optitrack, vicon, qualisys, ...")
    ap.add_argument("--mocap-host", default="192.168.1.100")
    ap.add_argument("--body", default="cf", help="rigid-body name in the mocap software")
    ap.add_argument("--controller", choices=list(CONTROLLER_ID), default="ftc")
    ap.add_argument("--no-fault", action="store_true")
    ap.add_argument("--fault-motor", type=int, choices=[1, 2, 3, 4], default=1)
    ap.add_argument("--fault-waypoint", type=int, default=2, help="trigger 0.5 s after this waypoint")
    ap.add_argument("--fault-key", action="store_true", help="trigger only with the f key")
    ap.add_argument("--no-inject", action="store_true", help="do not emulate the limit (evaluator does)")
    ap.add_argument("--inj-cap", type=float, default=0.25, help="thrust fraction for the emulated limit")
    ap.add_argument("--notify-delay", type=float, default=0.0)
    ap.add_argument("--mass", type=float, default=QuadParams.mass)
    args = ap.parse_args()
    use_fault = not args.no_fault and args.controller == "ftc"

    p = QuadParams(mass=args.mass)
    cfg = MPCConfig()
    mpc = PositionMPC(cfg)
    lo, hi = acc_limits(p)

    cflib.crtp.init_drivers()
    os.makedirs("logs", exist_ok=True)
    log_path = os.path.join("logs", time.strftime("flight_%Y%m%d_%H%M%S.csv"))
    state, lock = {}, threading.Lock()
    keys = KeyReader()
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf, open(log_path, "w", newline="") as fh:
        cf = scf.cf
        bridge = MocapBridge(cf, args.mocap_type, args.mocap_host, args.body)
        bridge.start()
        cf.param.set_value("stabilizer.estimator", "2")          # Kalman, fuses mocap pose
        cf.param.set_value("locSrv.extQuatStdDev", "0.06")
        cf.param.set_value("stabilizer.controller", CONTROLLER_ID[args.controller])
        if args.controller == "ftc":
            for name, val in (("ftc.faultMotor", "0"), ("ftc.injMotor", "0"), ("ftc.faultCap", "0.25"),
                              ("ftc.injCap", str(args.inj_cap))):
                cf.param.set_value(name, val)
        time.sleep(1.0)
        cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        cf.param.set_value("kalman.resetEstimation", "0")
        time.sleep(2.0)
        add_logs(cf, state, lock, args.controller == "ftc")
        time.sleep(0.5)
        with lock:
            start = np.array([state.get("stateEstimate.x", 0.0), state.get("stateEstimate.y", 0.0), 0.0])
        mission = Mission(start, waypoints=[(start[0] + w[0], start[1] + w[1], w[2]) for w in DEFAULT_WAYPOINTS])
        if hasattr(cf.platform, "send_arming_request"):
            cf.platform.send_arming_request(True)
            time.sleep(0.5)

        writer = csv.writer(fh)
        writer.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "rx", "ry", "rz", "phase", "roll", "pitch",
                         "yaw", "gyro_z", "f1", "f2", "f3", "f4", "mode", "fault_injected", "fault_notified"])
        t0 = time.time()
        t_fault = None
        injected = notified = False
        print("flying: f = fault now, q = KILL")
        try:
            while True:
                t = time.time() - t0
                k = keys.get()
                if k == "q":
                    print("KILL")
                    break
                with lock:
                    s = dict(state)
                pos = np.array([s.get("stateEstimate.x", 0), s.get("stateEstimate.y", 0), s.get("stateEstimate.z", 0)])
                vel = np.array([s.get("stateEstimate.vx", 0), s.get("stateEstimate.vy", 0), s.get("stateEstimate.vz", 0)])

                if use_fault and t_fault is None:
                    auto = (not args.fault_key) and t >= mission.waypoint_times[args.fault_waypoint] + 0.5
                    if auto or k == "f":
                        t_fault = t
                if t_fault is not None and not injected and not args.no_inject:
                    cf.param.set_value("ftc.injMotor", str(args.fault_motor))
                    injected = True
                if t_fault is not None and not notified and t >= t_fault + args.notify_delay:
                    cf.param.set_value("ftc.faultMotor", str(args.fault_motor))
                    mission.notify_fault(t, pos)
                    lo, hi = acc_limits(p, args.fault_motor, 0.25)
                    notified = True
                    print(f"fault on motor {args.fault_motor} at t={t:.2f}s: dropping yaw and landing")

                ref, phase = mission.ref_traj(t, cfg.horizon, cfg.dt)
                acc, p1, v1 = mpc.solve(pos, vel, ref, lo, hi)
                cf.commander.send_full_state_setpoint(p1, v1, acc, [0, 0, 0, 1], 0, 0, 0)

                writer.writerow([f"{t:.3f}", *pos, *vel, *ref[0], phase,
                                 s.get("stateEstimate.roll", 0), s.get("stateEstimate.pitch", 0),
                                 s.get("stateEstimate.yaw", 0), s.get("gyro.z", 0),
                                 s.get("ftc.f1", 0), s.get("ftc.f2", 0), s.get("ftc.f3", 0), s.get("ftc.f4", 0),
                                 s.get("ftc.mode", 0), int(injected), int(notified)])
                if phase in ("land", "fault_land") and pos[2] < 0.04:
                    print("landed")
                    break
                time.sleep(max(0.0, cfg.dt - ((time.time() - t0) - t)))
        except KeyboardInterrupt:
            print("KILL (Ctrl+C)")
        finally:
            cf.commander.send_stop_setpoint()
            cf.commander.send_notify_setpoint_stop()
            if args.controller == "ftc":
                cf.param.set_value("ftc.injMotor", "0")
                cf.param.set_value("ftc.faultMotor", "0")
            bridge.running = False
            keys.close()
    print(f"log saved to {log_path}")


if __name__ == "__main__":
    main()
