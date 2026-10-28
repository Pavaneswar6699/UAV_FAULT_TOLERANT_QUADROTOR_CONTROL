# Fault-tolerant quadrotor simulator

A small Python simulator of a Crazyflie-sized quadrotor flying the demo rectangle. Partway along
B → C, motor M1 is capped at 50% RPM (25% thrust). It compares:

- **stock**: a normal cascade controller with a Crazyflie-style mixer, never told about the fault
- **ours**: INDI attitude control + WLS motor allocation, told about the fault (reduced attitude, yaw released)

## Run it (Mac or Linux)

```bash
cd sim
/opt/homebrew/bin/python3.11 -m venv .venv          # any Python 3.10+ works
.venv/bin/pip install numpy scipy matplotlib pytest
.venv/bin/python run_sim.py                         # both controllers, fault on M1
.venv/bin/python run_sim.py --no-fault              # baseline flight
.venv/bin/python -m pytest -q                       # tests
```

Plots and CSV logs go to `results/`. To watch the flight in 3D: `.venv/bin/python export_viewer.py`, then `open results/flight_viewer.html`. Options: `--controller stock|ours|both`, `--fault-motor 1-4`,
`--fault-time`, `--side`, `--speed`, `--yaw-drag`, `--motor-tau` (`run_sim.py --help`).

## Files

| File | What it does |
|---|---|
| `quad.py` | Crazyflie parameters, motor layout (B matrix), rigid-body physics |
| `allocation.py` | Stock mixer and the WLS allocator |
| `control.py` | Stock controller and ours (INDI + WLS, fault mode) |
| `run_sim.py` | Rectangle mission, fault, settle → health check → continue or land, plots |
| `tests/test_core.py` | Checks the mixer, hover, and the fault split from slide 4 |

## What the parameters are

From the Crazyflie model in gym-pybullet-drones: mass 27 g, arm 39.7 mm, inertia, thrust-to-weight 2.25.

**Assumed, not measured. The result depends on these two:**

| Parameter | Default | Meaning | How to measure on the real drone |
|---|---|---|---|
| `motor_tau` | 0.03 s | how fast a motor responds | step a motor on the thrust stand |
| `yaw_drag` | 2e-5 | how fast it spins after the fault (~700 deg/s) | tethered fault test, read the gyro |

## Results so far (our controller, fault on M1)

| Motor response | Spin after fault | Result |
|---|---|---|
| 0.03 s | ~700 deg/s (default) | finishes the rectangle |
| 0.015 s (brushless-like) | ~700 deg/s | finishes the rectangle |
| 0.05 s | ~300 deg/s | finishes the rectangle |
| 0.05 s | ~1500 deg/s | **flips** |
| 0.015 s | ~1600 deg/s | lands safely (health check fails) |

The stock controller crashes within about a second of the fault in every case.

Fast spin with slow motors fails because the tilt corrections must change direction several times a
second and the motors can't keep up. Gain tuning alone did not fix it.

## Next steps

1. Measure `motor_tau` and the real spin rate as soon as the drone arrives.
2. Spin-aware control for the fast-spin case (compensate motor lag at the spin frequency).
3. Replace the position PD with MPC (CasADi) and add a wind estimate.
4. Port INDI + WLS to the firmware app and test in CrazySim.
