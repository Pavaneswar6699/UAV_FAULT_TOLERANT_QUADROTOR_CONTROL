"""Plot a real-flight log from hw_fly_mission.py.

    python scripts/plot_log.py logs/flight_20261120_153000.csv
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main(path):
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    t = d["t"].astype(float)
    fault = d["fault_notified"].astype(int)
    t_fault = t[np.argmax(fault > 0)] if fault.any() else None
    tilt = np.degrees(np.arccos(np.clip(np.cos(np.radians(d["roll"].astype(float)))
                                        * np.cos(np.radians(d["pitch"].astype(float))), -1, 1)))
    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0, 0].plot(d["x"], d["y"], label="flown")
    ax[0, 0].plot(d["rx"], d["ry"], "--", label="reference")
    ax[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Path (top view)")
    ax[0, 0].axis("equal")
    ax[0, 0].legend()
    ax[0, 1].plot(t, d["z"], label="z")
    ax[0, 1].plot(t, d["rz"], "--", label="z reference")
    ax[0, 1].set(xlabel="t [s]", ylabel="height [m]", title="Height")
    ax[0, 1].legend()
    ax[1, 0].plot(t, tilt, label="tilt [deg]")
    ax[1, 0].plot(t, np.abs(d["gyro_z"].astype(float)) / 57.3, label="|yaw rate| [rad/s]")
    ax[1, 0].set(xlabel="t [s]", title="Tilt and spin")
    ax[1, 0].legend()
    for k in ("f1", "f2", "f3", "f4"):
        ax[1, 1].plot(t, d[k], label=k)
    ax[1, 1].set(xlabel="t [s]", ylabel="commanded thrust [N]", title="Motor thrusts (ftc logs)")
    ax[1, 1].legend()
    if t_fault is not None:
        for a in (ax[0, 1], ax[1, 0], ax[1, 1]):
            a.axvline(t_fault, color="r", ls=":")
    fig.tight_layout()
    out = os.path.splitext(path)[0] + ".png"
    fig.savefig(out, dpi=110)
    print(f"saved {out}")


if __name__ == "__main__":
    main(sys.argv[1])
