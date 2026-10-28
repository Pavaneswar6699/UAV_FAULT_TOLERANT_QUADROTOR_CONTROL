"""Watch the simulated flights as a 3D animation.

Run run_sim.py first, then (from the sim folder):
  .venv/bin/python animate.py                          # opens a window, both drones side by side
  .venv/bin/python animate.py --slow 0.25              # quarter speed, to see the spin after the fault
  .venv/bin/python animate.py --start 8 --end 16       # only the part around the fault
  .venv/bin/python animate.py --save results/fault.gif # save a GIF instead of opening a window
"""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

RESULTS = Path(__file__).parent / "results"
ARM_DRAW = 0.18     # drawn arm length [m]; the real 4 cm arm would be invisible at this scale
# Motor positions in the body frame: M1 front-right, M2 back-right, M3 back-left, M4 front-left.
MOTOR_OFFSETS = np.array([[1, -1, 0], [-1, -1, 0], [-1, 1, 0], [1, 1, 0]]) / np.sqrt(2) * ARM_DRAW
COLORS = {"stock": "tab:gray", "ours": "tab:orange"}
TITLES = {"stock": "Stock controller", "ours": "Our controller (INDI + WLS)"}
WEAK = "tab:red"
HEALTHY = "#2b3440"


def load(name):
    path = RESULTS / f"log_{name}.csv"
    return np.loadtxt(path, delimiter=",") if path.exists() else None


class DronePanel:
    """One 3D view: the planned rectangle, the flown trail and the drone itself."""

    def __init__(self, ax, name, log, meta):
        self.ax, self.name, self.log, self.meta = ax, name, log, meta
        side = meta["side"]
        rect = np.array([[0, 0], [side, 0], [side, side], [0, side], [0, 0]])
        ax.plot(rect[:, 0], rect[:, 1], np.full(5, 0.8), "k--", lw=0.8, alpha=0.5)
        ax.set(xlim=(-0.4, side + 0.4), ylim=(-0.4, side + 0.4), zlim=(0, 1.2),
               xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
        ax.set_title(TITLES[name], color=COLORS[name], fontweight="bold")
        self.trail, = ax.plot([], [], [], color=COLORS[name], lw=1.2)
        self.shadow, = ax.plot([], [], [], color="k", lw=0.6, alpha=0.25)
        self.arms = [ax.plot([], [], [], color=HEALTHY, lw=2.5)[0] for _ in range(2)]
        self.motors = [ax.plot([], [], [], "o", color=HEALTHY)[0] for _ in range(4)]
        self.thrust, = ax.plot([], [], [], color="tab:blue", lw=1.5)
        self.text = ax.text2D(0.02, 0.97, "", transform=ax.transAxes, va="top", family="monospace", fontsize=9)

    def draw(self, t):
        log = self.log
        i = min(np.searchsorted(log[:, 0], t), len(log) - 1)
        row = log[i]
        pos, rot = row[1:4], row[13:22].reshape(3, 3)
        motors = pos + MOTOR_OFFSETS @ rot.T
        trail = log[: i + 1: 10]
        self.trail.set_data_3d(trail[:, 1], trail[:, 2], trail[:, 3])
        self.shadow.set_data_3d(trail[:, 1], trail[:, 2], np.zeros(len(trail)))
        for arm, (a, b) in zip(self.arms, [(0, 2), (1, 3)]):
            arm.set_data_3d(*np.stack([motors[a], motors[b]]).T)
        faulted = row[23] > 0.5
        weak = self.meta["fault_motor"] - 1
        for m, marker in enumerate(self.motors):
            marker.set_data_3d([motors[m, 0]], [motors[m, 1]], [motors[m, 2]])
            marker.set_markersize(3 + 6 * row[9 + m] / self.meta["hover_thrust"])
            marker.set_color(WEAK if faulted and m == weak else HEALTHY)
        tip = pos + rot[:, 2] * 0.2
        self.thrust.set_data_3d(*np.stack([pos, tip]).T)
        mode = self.meta["modes"][int(row[22])]
        status = f"t = {t:5.1f} s   {mode.upper()}\ntilt {row[7]:4.1f}°   spin {np.degrees(row[8]) + 0.0:5.0f}°/s"
        if t > log[-1, 0]:
            status += f"\n→ {self.meta['outcomes'][self.name]}"
        self.text.set_text(status)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slow", type=float, default=1.0, help="playback speed (0.25 = quarter speed)")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, help="default: end of the longest flight")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--save", help="save to a .gif instead of opening a window")
    args = ap.parse_args()

    meta = json.loads((RESULTS / "meta.json").read_text())
    logs = {name: load(name) for name in ("stock", "ours")}
    logs = {k: v for k, v in logs.items() if v is not None and k in meta["outcomes"]}
    end = args.end if args.end is not None else max(l[-1, 0] for l in logs.values()) + 1.0
    times = np.arange(args.start, end, args.slow / args.fps)

    fig = plt.figure(figsize=(6 * len(logs), 6))
    panels = [DronePanel(fig.add_subplot(1, len(logs), k + 1, projection="3d"), name, log, meta)
              for k, (name, log) in enumerate(logs.items())]
    fault = f"fault on M{meta['fault_motor']} at t = {meta['fault_time']:.1f} s" if meta["fault_time"] else "no fault"
    fig.suptitle(f"Rectangle mission · {fault} · red motor = weak · blue line = thrust direction"
                 + (f" · {args.slow}x speed" if args.slow != 1 else ""), fontsize=10)

    def update(frame):
        for panel in panels:
            panel.draw(times[frame])
        return []

    anim = FuncAnimation(fig, update, frames=len(times), interval=1000 / args.fps, blit=False)
    if args.save:
        anim.save(args.save, writer=PillowWriter(fps=args.fps))
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
