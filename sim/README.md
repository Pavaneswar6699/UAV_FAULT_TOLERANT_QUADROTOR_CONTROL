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
| no wind compensation | PASS, 0.30 m (limit 0.3) | 0.40 m |
| Kalman estimate fed to the controller | PASS, 0.03 m | 0.16 m |

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
| 0.05 s | ~1500 deg/s | passes the health check, then **flips** while continuing |
| 0.015 s | ~2400 deg/s (past the gyro limit) | lands safely (health check fails) |

The stock controller crashes within about a second of the fault in every case.

While the drone spins, the tilt corrections must change direction several times a second and the
motors deliver them late. The controller now leads the roll/pitch torque by the motors' phase lag at
the spin rate (`IndiWlsController` in `control.py`), which is what lets it fly CrazySim's slower motors.
Very fast spin with slow motors still fails.

## In CrazySim (the real firmware)

The same controller, ported to C (`../firmware/indi_wls.c`, checked against this Python version by
`tests/test_firmware_core.py`), runs inside the Crazyflie firmware. How to run it: `LINUX_SETUP.md`.
Fault on M1 midway along B -> C, CrazySim started with `--yaw-drag 2e-5`:

| CrazySim model | Thrust left after the fault / weight | Stock firmware | Ours |
|---|---|---|---|
| `cf21B_500` (2.1 Brushless) | 1.17 | crashes | **finishes the rectangle** |
| `cf2x_T350` (thrust upgrade) | 1.21 | crashes | stays level, health check fails, **lands safely** |
| `cf2x_L250` (standard 2.x) | 0.96 | crashes | stays level but sinks to the ground |

- The standard Crazyflie 2.x can't hold its height on three and a quarter motors: no controller can.
- CrazySim has no yaw damping of its own. Without `--yaw-drag` the spin grows until the gyro saturates
  (2000 deg/s) and every controller flips. The real drone has yaw drag; its size is the `yaw_drag`
  above, still to be measured.
- CrazySim's motors are 2-5x slower than `motor_tau` here (0.065-0.145 s); the gains in `control.py`
  were retuned so both simulators fly with the same values.

## Next steps

1. Measure `motor_tau` and the real spin rate as soon as the drone arrives.
2. Tighten position hold while spinning in CrazySim (the T350 health check fails by a few cm).
3. Replace the position PD with MPC (CasADi) and add a wind estimate.
