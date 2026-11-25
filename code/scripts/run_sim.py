"""Simulate the mission with and without the 50% RPM fault and compare controllers.

    python scripts/run_sim.py                  # all three scenarios, plots in results/
    python scripts/run_sim.py --scenario ftc   # only our controller with the fault
    python scripts/run_sim.py --mass 0.0325 --fmax 0.18   # thrust-upgrade-kit numbers
"""
import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ftqc.params import QuadParams
from ftqc.sim import simulate

SCENARIOS = {
    "baseline": dict(mode="ftc", fault_motor=0),        # healthy mission
    "pid": dict(mode="pid", fault_motor=1),             # fault, stock-style controller
    "ftc": dict(mode="ftc", fault_motor=1),             # fault, our controller
}


def save_csv(path, log):
    keys = list(log.keys())
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(keys)
        for row in zip(*[log[k] for k in keys]):
            w.writerow([f"{v:.5f}" if isinstance(v, (float, np.floating)) else v for v in row])


def plot(name, log, summary, out_dir):
    t = log["t"]
    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0, 0].plot(log["x"], log["y"], label="flown")
    ax[0, 0].plot(log["rx"], log["ry"], "--", label="reference")
    ax[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Path (top view)")
    ax[0, 0].axis("equal")
    ax[0, 0].legend()
    ax[0, 1].plot(t, log["z"], label="z")
    ax[0, 1].plot(t, log["rz"], "--", label="z reference")
    ax[0, 1].set(xlabel="t [s]", ylabel="height [m]", title="Height")
    ax[0, 1].legend()
    ax[1, 0].plot(t, log["tilt"], label="tilt [deg]")
    ax[1, 0].plot(t, np.abs(log["yaw_rate"]), label="|yaw rate| [rad/s]")
    ax[1, 0].set(xlabel="t [s]", title="Tilt and spin")
    ax[1, 0].legend()
    for k in ("f1", "f2", "f3", "f4"):
        ax[1, 1].plot(t, log[k], label=k)
    ax[1, 1].set(xlabel="t [s]", ylabel="motor thrust [N]", title="Motor thrusts")
    ax[1, 1].legend()
    if summary.get("t_fault") is not None:
        for a in (ax[0, 1], ax[1, 0], ax[1, 1]):
            a.axvline(summary["t_fault"], color="r", ls=":", label="fault")
    fig.suptitle(f"{name}: {summary['result']}")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"{name}.png"), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=["all", *SCENARIOS], default="all")
    ap.add_argument("--mass", type=float, default=QuadParams.mass)
    ap.add_argument("--fmax", type=float, default=QuadParams.f_max)
    ap.add_argument("--fault-motor", type=int, default=1)
    ap.add_argument("--notify-delay", type=float, default=0.0)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "results"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    p = QuadParams(mass=args.mass, f_max=args.fmax)
    print(f"mass {p.mass*1000:.1f} g, f_max {p.f_max:.3f} N/motor, thrust-to-weight {4*p.f_max/p.weight:.2f}")
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    results = {}
    for name in names:
        kw = dict(SCENARIOS[name])
        if kw["fault_motor"]:
            kw["fault_motor"] = args.fault_motor
        log, summary = simulate(p=p, notify_delay=args.notify_delay, **kw)
        save_csv(os.path.join(args.out, f"{name}.csv"), log)
        plot(name, log, summary, args.out)
        results[name] = summary
        print(f"{name:9s} -> {summary['result']:8s} max tilt after fault {summary['max_tilt_after_fault']:6.1f} deg, "
              f"max spin {summary['max_yaw_rate']:5.1f} rad/s, touchdown speed "
              f"{summary.get('touchdown_speed', float('nan')):.2f} m/s")
    with open(os.path.join(args.out, "summary.json"), "w") as fh:
        json.dump(results, fh, indent=2)
    print(f"plots, CSVs and summary.json written to {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()
