"""Turn the latest simulation run into a 3D flight viewer page.

  .venv/bin/python run_sim.py          # any experiment
  .venv/bin/python export_viewer.py    # writes results/flight_viewer.html
  open results/flight_viewer.html      # watch it in your browser
"""
import json
from pathlib import Path

import numpy as np

from quad import GRAVITY, QuadParams

HERE = Path(__file__).parent
RESULTS = HERE / "results"
FRAME_DT = 0.02                      # 50 frames per second of flight
# Sim world is z-up (x east, y north); three.js is y-up. This maps sim -> three: (x, y, z) -> (x, z, -y).
TO_THREE = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)


def quaternion(rot: np.ndarray) -> list:
    """Rotation matrix -> quaternion [x, y, z, w]."""
    w = np.sqrt(max(0.0, 1.0 + np.trace(rot))) / 2.0
    if w > 1e-6:
        return [(rot[2, 1] - rot[1, 2]) / (4 * w), (rot[0, 2] - rot[2, 0]) / (4 * w),
                (rot[1, 0] - rot[0, 1]) / (4 * w), w]
    i = int(np.argmax(np.diag(rot)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(max(0.0, 1.0 + rot[i, i] - rot[j, j] - rot[k, k])) * 2.0
    q = [0.0, 0.0, 0.0, (rot[k, j] - rot[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (rot[j, i] + rot[i, j]) / s, (rot[k, i] + rot[i, k]) / s
    return q


def export_run(log: np.ndarray) -> dict:
    step = max(1, int(round(FRAME_DT / (log[1, 0] - log[0, 0]))))
    rows = np.vstack([log[::step], log[-1:]])
    pos = rows[:, 1:4] @ TO_THREE.T
    quats = [quaternion(TO_THREE @ r.reshape(3, 3) @ TO_THREE.T) for r in rows[:, 13:22]]
    return {
        "t": np.round(rows[:, 0], 3).tolist(),
        "p": np.round(pos, 4).ravel().tolist(),
        "q": np.round(np.array(quats), 4).ravel().tolist(),
        "f": np.round(rows[:, 9:13] / GRAVITY * 1000, 2).ravel().tolist(),   # grams
        "tilt": np.round(rows[:, 7], 1).tolist(),
        "spin": np.round(np.degrees(rows[:, 8]), 0).tolist(),
        "mode": rows[:, 22].astype(int).tolist(),
        "faulted": rows[:, 23].astype(int).tolist(),
    }


def main():
    meta = json.loads((RESULTS / "meta.json").read_text())
    p = QuadParams()
    d = p.arm / np.sqrt(2)
    motors_sim = np.array([[d, -d, 0], [-d, -d, 0], [-d, d, 0], [d, d, 0]])   # M1 FR, M2 BR, M3 BL, M4 FL
    side = meta["side"]
    corners = np.array([[0, 0, 0.8], [side, 0, 0.8], [side, side, 0.8], [0, side, 0.8]])
    runs = {}
    for name in ("ours", "stock"):
        path = RESULTS / f"log_{name}.csv"
        if path.exists() and name in meta["outcomes"]:
            runs[name] = export_run(np.loadtxt(path, delimiter=","))
    data = {
        "meta": {
            **meta,
            "f_max_g": round(p.f_max / GRAVITY * 1000, 2),
            "motors": np.round(motors_sim @ TO_THREE.T, 5).tolist(),
            "corners": np.round(corners @ TO_THREE.T, 3).tolist(),
        },
        "runs": runs,
    }
    html = (HERE / "viewer_template.html").read_text().replace("/*__DATA__*/null", json.dumps(data, separators=(",", ":")))
    out = RESULTS / "flight_viewer.html"
    out.write_text(html)
    print(f"wrote {out} ({out.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
