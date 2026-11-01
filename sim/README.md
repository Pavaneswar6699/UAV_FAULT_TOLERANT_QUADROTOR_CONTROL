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
| `wind_kf.py` | Linear Kalman filter that estimates the wind from position and velocity |
| `tests/test_core.py` | Checks the mixer, hover, and the fault split from slide 4 |
| `tests/test_wind_kf.py` | Checks the wind estimate converges and removes the wind offset |

## Wind estimate (Kalman filter)

State = position, velocity, wind acceleration. It measures position and velocity (from the Crazyflie's
onboard EKF on the real drone) and uses the motor commands as the known input. On the real system it
runs on the laptop and its estimate goes into the MPC; here it is added to the position controller.

```bash
.venv/bin/python run_sim.py --controller ours --wind 2 0             # estimate only
.venv/bin/python run_sim.py --controller ours --wind 2 0 --wind-ff   # controller leans into it
```

| 2 m/s wind, fault on M1 | Health check after the fault | Max position error |
|---|---|---|
| no wind compensation | PASS, 0.25 m (limit 0.3) | 0.33 m |
| Kalman estimate fed to the controller | PASS, 0.03 m | 0.12 m |

The estimate reaches the true wind force (20 mN) within about 2 s and stays on it through the fault and
the spin (`results/wind_estimate.png`). Tuning constants are at the top of `wind_kf.py`; set `POS_STD`
and `VEL_STD` to the real Loco noise once measured.

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
