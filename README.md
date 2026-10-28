# Fault-Tolerant Quadrotor Control

**Project 6 · Introduction to UAV Design · Team LUGER**

A quadrotor that keeps flying its path when one motor drops to 50% RPM. Platform: Crazyflie
nano quadrotor. Demo: one rectangular path on a football ground.

**Approach.** Half the RPM leaves 25% of the thrust, and a Crazyflie's thrust-to-weight (≈2.2–3.5)
is below the 4:1 needed to stay level *and* hold heading. So after the fault we release heading
(the drone spins) and keep it level and on the path:

- **MPC** (Model Predictive Control) on the ground station plans the motion within the new limits
- **INDI** (Incremental Nonlinear Dynamic Inversion) on the drone keeps the tilt right
- **WLS** (Weighted Least Squares) allocation shares the work across the 4 motors, each within its own limit

## Repository

| Path | What it is |
|---|---|
| `Fault-Tolerant Quadrotor Control - Crazyflie.pptx` | Approved presentation (objectives, framework, deliverables, components) |
| `Team_LUGER_Slide_Guide.pdf` | What each slide says, plus likely questions |
| `WhatsApp Image …jpeg` | The project brief and general expectations |
| `Fault-Tolerant Quadrotor Control — Project Presentation.pptx`, `Team_LUGER_Presentation_Script.pdf` | Earlier PX4-based version (superseded) |
| `sim/` | Python simulator, 3D flight viewer, tests, learning guide |
| `sim/crazysim/` | Scripts for CrazySim (the real Crazyflie firmware in simulation) |

## Quick start (Ubuntu)

```bash
git clone https://github.com/Pavaneswar6699/UAV_FAULT_TOLERANT_QUADROTOR_CONTROL.git
cd UAV_FAULT_TOLERANT_QUADROTOR_CONTROL/sim
sudo apt install -y python3-venv python3-tk
python3 -m venv .venv
.venv/bin/pip install numpy scipy matplotlib pytest
.venv/bin/python -m pytest -q           # 5 passed
.venv/bin/python run_sim.py             # stock controller crashes, ours finishes the rectangle
.venv/bin/python export_viewer.py && xdg-open results/flight_viewer.html
```

Then follow [`sim/LINUX_SETUP.md`](sim/LINUX_SETUP.md) to install CrazySim, and
[`sim/LEARN.md`](sim/LEARN.md) to learn how the simulator works.
