"""Bench calibration: find the motor command that gives 50% of the maximum RPM.

SAFETY: clamp the Crazyflie down, props on, keep fingers away. Spins ONE motor.

    python scripts/hw_calibrate_motor.py --motor 1

For each command level the motor spins; point the optical tachometer at the
reflective tape on that motor and type the RPM reading. The script saves
calibration/motor<N>.csv (columns motor,cmd,rpm) and prints:
  * the raw command (0..65535) for 50% of max RPM
  * the matching thrust fraction (~(0.5)^2 = 0.25), to use as ftc.injCap / ftc.faultCap
"""
import argparse
import csv
import os
import time

import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

LEVELS = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uri", default="radio://0/80/2M/E7E7E7E7E7")
    ap.add_argument("--motor", type=int, choices=[1, 2, 3, 4], required=True)
    ap.add_argument("--spin-time", type=float, default=2.0)
    args = ap.parse_args()

    cflib.crtp.init_drivers()
    rows = []
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        if hasattr(cf, "platform") and hasattr(cf.platform, "send_arming_request"):
            cf.platform.send_arming_request(True)
            time.sleep(0.5)
        name = f"motorPowerSet.m{args.motor}"
        cf.param.set_value("motorPowerSet.enable", "1")
        try:
            for level in LEVELS:
                cmd = int(level * 65535)
                cf.param.set_value(name, str(cmd))
                time.sleep(args.spin_time)
                rpm = float(input(f"command {cmd:5d} ({level:.0%}): tachometer RPM? "))
                rows.append((args.motor, cmd, rpm))
        finally:
            cf.param.set_value(name, "0")
            cf.param.set_value("motorPowerSet.enable", "0")

    os.makedirs("calibration", exist_ok=True)
    path = os.path.join("calibration", f"motor{args.motor}.csv")
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["motor", "cmd", "rpm"])
        w.writerows(rows)

    cmds = np.array([r[1] for r in rows], dtype=float)
    rpms = np.array([r[2] for r in rows], dtype=float)
    rpm_max = rpms.max()
    cmd_half = float(np.interp(0.5 * rpm_max, rpms, cmds))
    print(f"\nsaved {path}")
    print(f"max RPM {rpm_max:.0f}; 50% RPM = {0.5 * rpm_max:.0f} at command {cmd_half:.0f} "
          f"({cmd_half / 65535:.1%} of full command)")
    print("thrust fraction at 50% RPM ~ 0.25 (thrust ~ RPM^2): use ftc.faultCap = 0.25, and set "
          "ftc.injCap so the tachometer reads 50% RPM during a tethered test")


if __name__ == "__main__":
    main()
