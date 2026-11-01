"""Fly the demo rectangle with a mid-path motor fault, in CrazySim or on the real Crazyflie.

Start CrazySim first (see LINUX_SETUP.md), then:
  python3 crazysim/rectangle_cflib.py --controller ours     # our INDI + WLS controller
  python3 crazysim/rectangle_cflib.py --controller stock    # the firmware's PID controller
  python3 crazysim/rectangle_cflib.py --no-fault            # baseline flight
  python3 crazysim/rectangle_cflib.py --hover 5             # hover 5 s instead of the rectangle
--model must match the -m given to the CrazySim launch script (default cf2x_L250).
Real drone: --uri radio://0/80/2M/E7E7E7E7E7 --model cf2x_P250 --mass <measured kg> (firmware/README.md).
Needs the patched firmware (crazysim/firmware.patch). Restart CrazySim after a crash.
"""
import argparse
import csv
import math
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "udp://127.0.0.1:19850"     # CrazySim's first drone
HEIGHT = 0.8                      # [m]
SIDE = 1.5                        # [m]
LEG_TIME = 5.0                    # seconds per side (0.3 m/s)
CORNERS = [(SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE), (0.0, 0.0)]   # A -> B -> C -> D -> A
CRASH_TILT_DEG = 60.0
SETTLE_S = 2.5                    # after the fault: hold position, then check health (as in run_sim.py)
CHECK_WINDOW_S = 1.0
MAX_POS_ERR = 0.3                 # [m]
MAX_TILT_DEG = 20.0
RESULTS = Path(__file__).resolve().parent.parent / "results"
STOCK_CONTROLLER = 1             # stabilizer.controller: PID. Ours reports its own id (indiwls.controllerId)

# CrazySim drone models (drone-models params.toml and MJCF files)
MODELS = {
    "cf2x_L250": dict(mass=0.0319, fMax=0.12, armD=0.0325, kappa=0.00735086, ixx=1.68e-05, iyy=1.68e-05, izz=2.98e-05, motorTau=0.1452),
    "cf2x_P250": dict(mass=0.0318, fMax=0.12, armD=0.0325, kappa=0.00699289, ixx=1.68e-05, iyy=1.68e-05, izz=2.98e-05, motorTau=0.1297),
    "cf2x_T350": dict(mass=0.0379, fMax=0.18, armD=0.0325, kappa=0.00535599, ixx=1.68e-05, iyy=1.68e-05, izz=2.98e-05, motorTau=0.08808),
    "cf21B_500": dict(mass=0.04338, fMax=0.2, armD=0.03536, kappa=0.00593893, ixx=2.5e-05, iyy=2.8e-05, izz=4.9e-05, motorTau=0.06486),
}


class FlightLog:
    """Collects the firmware's state at 50 Hz and spots a crash."""

    def __init__(self):
        self.rows = []
        self.motors = (0, 0, 0, 0)
        self.yaw_rate = 0.0
        self.fault_on = False
        self.airborne = False
        self.crashed = False

    def start(self, cf: Crazyflie) -> None:
        pose = LogConfig("pose", period_in_ms=20)
        for name in ("x", "y", "z", "roll", "pitch", "yaw"):
            pose.add_variable(f"stateEstimate.{name}", "float")
        motors = LogConfig("motors", period_in_ms=20)
        motors.add_variable("gyro.z", "float")
        for k in range(1, 5):
            motors.add_variable(f"fault.m{k}", "uint16_t")
        for conf, cb in ((pose, self._pose), (motors, self._motors)):
            cf.log.add_config(conf)
            conf.data_received_cb.add_callback(cb)
            conf.start()

    def _motors(self, _ts, data, _conf):
        self.yaw_rate = data["gyro.z"]
        self.motors = tuple(data[f"fault.m{k}"] for k in range(1, 5))

    def _pose(self, ts, data, _conf):
        x, y, z = (data[f"stateEstimate.{n}"] for n in "xyz")
        roll, pitch = data["stateEstimate.roll"], data["stateEstimate.pitch"]
        tilt = math.degrees(math.acos(min(1.0, math.cos(math.radians(roll)) * math.cos(math.radians(pitch)))))
        self.rows.append((ts / 1000.0, x, y, z, roll, pitch, data["stateEstimate.yaw"], tilt,
                          self.yaw_rate, *self.motors, int(self.fault_on)))
        if self.airborne and (tilt > CRASH_TILT_DEG or z < 0.1):
            self.crashed = True

    def position(self):
        return self.rows[-1][1:4]

    def save(self, path: Path) -> None:
        path.parent.mkdir(exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg", "tilt_deg", "yaw_rate_dps",
                        "pwm1", "pwm2", "pwm3", "pwm4", "faulted"])
            w.writerows(self.rows)


def wait(log: FlightLog, seconds: float) -> bool:
    """Sleep while watching for a crash. Returns False if the drone crashed."""
    end = time.time() + seconds
    while time.time() < end:
        if log.crashed:
            return False
        time.sleep(0.02)
    return not log.crashed


def health_check(log: FlightLog, hold) -> tuple[bool, str]:
    """Same test as run_sim.py: over the last second, position error and tilt both small."""
    t_end = log.rows[-1][0]
    recent = [r for r in log.rows if r[0] >= t_end - CHECK_WINDOW_S]
    worst_err = max(math.dist(r[1:4], hold) for r in recent)
    worst_tilt = max(r[7] for r in recent)
    passed = worst_err < MAX_POS_ERR and worst_tilt < MAX_TILT_DEG
    return passed, f"{'PASS' if passed else 'FAIL'} (pos err {worst_err:.2f} m, tilt {worst_tilt:.1f} deg)"


def inject_fault(cf: Crazyflie, log: FlightLog, args) -> None:
    cf.param.set_value("fault.motor", str(args.fault_motor))
    if args.controller == "ours":
        cf.param.set_value("indiwls.faultMotor", str(args.fault_motor))   # notify_fault()
    log.fault_on = True
    print(f"FAULT: M{args.fault_motor} capped at {args.fault_cap:.0%} thrust")


def hover(cf: Crazyflie, log: FlightLog, args) -> str:
    """Take off, hover in place, fault halfway if asked, land. For first flights on the real drone."""
    hl = cf.high_level_commander
    hl.takeoff(args.hover_height, 2.0)
    time.sleep(3.0)
    log.airborne = True
    if not wait(log, args.hover / 2):
        return "crashed"
    if not args.no_fault:
        inject_fault(cf, log, args)
        if args.land_on_fault:
            log.airborne = False
            hl.land(0.0, args.land_on_fault)
            time.sleep(args.land_on_fault + 0.5)
            hl.stop()
            return "landed after the fault"
    if not wait(log, args.hover / 2):
        return "crashed"
    log.airborne = False
    hl.land(0.0, 3.0)
    time.sleep(3.5)
    hl.stop()
    return "hovered and landed"


def fly(cf: Crazyflie, log: FlightLog, args) -> str:
    hl = cf.high_level_commander
    hl.takeoff(HEIGHT, 2.0)
    time.sleep(3.0)
    log.airborne = True
    for leg, (x, y) in enumerate(CORNERS):
        print(f"going to ({x:.1f}, {y:.1f})")
        hl.go_to(x, y, HEIGHT, 0.0, LEG_TIME)
        if leg != 1 or args.no_fault:
            if not wait(log, LEG_TIME + 0.5):
                return "crashed"
            continue
        # Leg B -> C: the fault, then settle -> health check -> continue at half speed or land
        if not wait(log, args.fault_delay):
            return "crashed"
        inject_fault(cf, log, args)
        hold = (*log.position()[:2], HEIGHT)
        hl.go_to(*hold, 0.0, 1.0)
        if not wait(log, SETTLE_S):
            return "crashed"
        passed, decision = health_check(log, hold)
        print(f"health check: {decision}")
        if not passed:
            print("landing where it is")
            log.airborne = False
            hl.land(0.0, 4.0)
            time.sleep(4.5)
            hl.stop()
            return "landed safely before finishing"
        here = hold[:2]
        for x2, y2 in CORNERS[leg:]:
            print(f"continuing at half speed to ({x2:.1f}, {y2:.1f})")
            duration = 2 * LEG_TIME * math.dist(here, (x2, y2)) / SIDE
            hl.go_to(x2, y2, HEIGHT, 0.0, duration)
            if not wait(log, duration + 0.5):
                return "crashed"
            here = (x2, y2)
        break
    log.airborne = False
    hl.land(0.0, 2.5)
    time.sleep(3.0)
    hl.stop()
    return "finished the rectangle and landed at A"


def summary(log: FlightLog, outcome: str) -> None:
    after = [r for r in log.rows if r[-1]] or log.rows
    print("CRASHED (flipped or hit the ground)" if outcome == "crashed" else outcome)
    print(f"    max tilt {max(r[7] for r in log.rows):.1f} deg · "
          f"peak spin {max(abs(r[8]) for r in log.rows):.0f} deg/s (gyro limit 2000)")
    if any(r[-1] for r in log.rows):
        print(f"    after the fault: lowest height {min(r[3] for r in after):.2f} m · "
              f"max tilt {max(r[7] for r in after):.1f} deg")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controller", choices=["stock", "ours"], default="ours")
    ap.add_argument("--model", choices=list(MODELS), default="cf2x_L250", help="same as CrazySim's -m")
    ap.add_argument("--mass", type=float, help="override the model's mass [kg] (CrazySim: also pass -M)")
    ap.add_argument("--uri", default=URI, help="real drone: radio://0/80/2M/E7E7E7E7E7")
    ap.add_argument("--hover", type=float, metavar="SECONDS", help="hover in place instead of the rectangle")
    ap.add_argument("--hover-height", type=float, default=0.5, help="[m] for --hover (default 0.5)")
    ap.add_argument("--land-on-fault", type=float, metavar="SECONDS", help="with --hover: land right after the fault, taking this long")
    ap.add_argument("--fault-motor", type=int, default=1, choices=[1, 2, 3, 4])
    ap.add_argument("--fault-cap", type=float, default=0.25, help="thrust left on the faulty motor (default 0.25 = 50%% RPM)")
    ap.add_argument("--fault-delay", type=float, default=LEG_TIME / 2, help="seconds into leg B -> C (default: halfway)")
    ap.add_argument("--no-fault", action="store_true")
    args = ap.parse_args()

    cflib.crtp.init_drivers()
    model = dict(MODELS[args.model], **({"mass": args.mass} if args.mass else {}))
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        cf.param.set_value("fault.motor", "0")
        cf.param.set_value("fault.cap", str(args.fault_cap))
        cf.param.set_value("indiwls.faultMotor", "0")
        cf.param.set_value("indiwls.faultCap", str(args.fault_cap))
        for name, value in model.items():
            cf.param.set_value(f"indiwls.{name}", str(value))
        ours = int(cf.param.get_value("indiwls.controllerId"))
        cf.param.set_value("stabilizer.controller", str(ours if args.controller == "ours" else STOCK_CONTROLLER))
        print(f"controller: {args.controller} · model: {args.model} · mass {model['mass'] * 1000:.1f} g")
        log = FlightLog()
        log.start(cf)
        cf.supervisor.send_arming_request(True)   # recent firmware needs arming before flight
        time.sleep(1.0)
        try:
            outcome = hover(cf, log, args) if args.hover else fly(cf, log, args)
        except KeyboardInterrupt:              # Ctrl+C: motors off now
            cf.high_level_commander.stop()
            print("STOPPED by Ctrl+C: motors off")
            outcome = "stopped"
        if outcome == "crashed":
            time.sleep(2.0)                        # record the fall
            cf.high_level_commander.stop()
        cf.param.set_value("fault.motor", "0")
        cf.param.set_value("indiwls.faultMotor", "0")

    fault = "nofault" if args.no_fault else f"fault_m{args.fault_motor}"
    where = "crazysim" if args.uri.startswith("udp://") else "real"
    path = RESULTS / f"{where}_{args.controller}_{args.model}_{'hover_' if args.hover else ''}{fault}.csv"
    log.save(path)
    summary(log, outcome)
    print(f"log saved in {path}")


if __name__ == "__main__":
    main()
