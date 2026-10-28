# Learn the simulator, step by step

This guide walks through the simulator in the order a real flight happens. Each lesson has
**the idea**, **where it is in the code**, and **try it**. Run everything from the `sim` folder.

```
          ┌──────────── every 2 ms (500 Hz) ─────────────┐
 sensors ─▶ controller ─▶ motor commands ─▶ physics (evsery 1 ms) ─▶ new state ─▶ sensors
              ▲
     mission (where to go) + fault message
```

## Lesson 0 · Watch it first

```bash
.venv/bin/python run_sim.py                       # simulate both controllers (≈4 s)
.venv/bin/python animate.py --start 8 --end 20 --slow 0.5
```
A window opens with two drones. Around t = 9.5 s, motor M1 turns **red** (it's now weak). Watch:
- **Stock (grey):** it tips toward the red motor and crashes within about a second.
- **Ours (orange):** it starts spinning (watch the arms rotate), but the **blue line** (thrust
  direction) stays nearly vertical, and it carries on along the rectangle.

Bigger motor dots mean more thrust. You'll see the red motor and the one diagonally opposite it
shrink, and the other two grow. That's slide 4 happening.

Also open `results/compare.png` and `results/motors_ours.png`.

**The 3D viewer (best way to watch):**
```bash
.venv/bin/python export_viewer.py        # turns the latest run into a web page
open results/flight_viewer.html          # opens it in your browser
```
Press **Jump to fault**, set speed to **¼×**, and drag to rotate the camera (scroll to zoom).
The status cards show each drone's tilt, spin, height and the four motor thrusts in grams. The red
line on a motor bar is the weak motor's 25% cap. Run any experiment from Lesson 5, then
`export_viewer.py` again to watch that one.

## Lesson 1 · The physics (`quad.py`)

**Idea.** A quadrotor is a rigid body pushed by 4 thrusts. Each step we:
1. move each motor's thrust a little toward its command (motors aren't instant: `motor_tau`);
2. turn the 4 thrusts into one push and three twists with the **B matrix**;
3. push → acceleration (Newton), twists → angular acceleration (Euler);
4. update speed, position, rotation rate and orientation.

**Code.** `allocation_matrix()` is B. `step()` does 1–4. The fault is applied in `step()` through
`motor_cap`: the physics simply won't let motor k go above 25%.

| Row of B | M1 FR | M2 BR | M3 BL | M4 FL | Meaning |
|---|---|---|---|---|---|
| T | +1 | +1 | +1 | +1 | all push up |
| τx (roll) | −d | −d | +d | +d | left side up vs right side |
| τy (pitch) | −d | +d | +d | −d | back up vs front |
| τz (yaw) | −κ | +κ | −κ | +κ | diagonals M1, M3 twist one way; M2, M4 the other |

**Try it.**
```bash
.venv/bin/python
>>> from quad import *
>>> p = QuadParams()
>>> p.f_max / GRAVITY * 1000          # max thrust per motor in grams  → 15.19
>>> p.mass * GRAVITY / 4 / GRAVITY * 1000   # thrust per motor to hover → 6.75
>>> allocation_matrix(p)
```
Exercise: change `thrust_to_weight` to 4.0 in `QuadParams` and recompute. What is 25% of the new
f_max, and is it above 6.75 g? (That's the 4:1 argument from slide 3.)

## Lesson 2 · Sharing work between motors (`allocation.py`)

**Idea.** The controller asks for [lift, roll, pitch, yaw]. Something must choose 4 motor thrusts.
- `stock_mix()` inverts B. If a motor goes over max, it lowers **all** motors equally. It assumes
  every motor is equally strong.
- `wls_allocate()` finds the thrusts **closest** to the request, keeps each motor **within its own
  limit**, and follows **priorities** (`NORMAL_WEIGHTS`: roll and pitch 100, lift 10, yaw 1).

**Try it.**
```bash
.venv/bin/python
>>> import numpy as np
>>> from quad import *; from allocation import *
>>> p = QuadParams(); hover = np.array([p.mass*GRAVITY, 0, 0, 0])
>>> upper = np.array([0.25, 1, 1, 1]) * p.f_max              # M1 is weak
>>> wls_allocate(hover, p, upper, NORMAL_WEIGHTS) / GRAVITY * 1000
array([3.8 , 9.64, 3.8 , 9.64])                             # weak + diagonal low, others high
>>> wls_allocate(hover, p, upper, np.ones(4)) / GRAVITY * 1000
array([3.8 , 6.75, 6.75, 6.75])                             # equal priorities: like the stock mixer
```
With equal priorities you lose about 3 g of lift and get a twist toward M1, so the drone tilts.
**The priorities are what keep it level.**

## Lesson 3 · The controllers (`control.py`)

The controller works in three layers, slow to fast:

1. **Where to push** (`thrust_vector`): position error → wanted acceleration → a thrust direction
   `n_des` and an amount `thrust`. Tilt is limited (25°, or 15° after the fault).
2. **How to turn** (`full_attitude_rates` / `reduced_attitude_rates`): the turn rates needed to
   point the drone at `n_des`. In fault mode, `reduced_attitude_rates` ignores heading entirely.
3. **INDI** (in `IndiWlsController.__call__`): the key lines are
   ```python
   w_dot_f = (w_f - self.w_f) / dt                       # measured angular acceleration (filtered)
   tau_cmd = self.tau_f + p.inertia @ (nu - w_dot_f)     # current torque + correction for the difference
   ```
   It doesn't need to know a motor is weak. If the drone rotates differently than expected, the
   gyro shows it and the difference gets corrected.

`StockController` uses the same layers 1 and 2, but a model-based rate loop and `stock_mix`, and
it ignores the fault message.

**What changes at the fault** (`notify_fault` → `_limits`): motor k's upper limit becomes 25%,
the yaw weight fades to 0.001 over 0.2 s, thrust and tilt limits drop.

## Lesson 4 · The mission (`run_sim.py`)

`Mission.update()` is a small state machine: `hold → fly → (fault) settle → health check →
continue or land`. `simulate()` is the main loop: every 1 ms it steps the physics; every 2 ms it
reads noisy sensors, asks the mission for a reference, and runs the controller. At `fault_time`
it caps the motor in the physics **and** tells the controller and the mission. That's the
"evaluator caps the motor and the laptop sends the message" from slide 8.

## Lesson 5 · Experiments

Run each, then look at `results/compare.png` (and `animate.py` if you want to watch).

| Command | What to look for |
|---|---|
| `run_sim.py --no-fault` | both fly the rectangle the same way: the baseline |
| `run_sim.py --fault-motor 2` | a different weak motor: the low pair becomes M2 + M4 |
| `run_sim.py --fault-time 4` | fault on the first side instead |
| `run_sim.py --speed 0.5` | faster flight: bigger tilt, larger errors |
| `run_sim.py --motor-tau 0.05 --yaw-drag 4.6e-6` | slow motors + fast spin: **ours flips too** |
| `run_sim.py --motor-tau 0.015 --yaw-drag 4.6e-6` | faster motors: survives but fails the health check and lands |

The last two show the open problem: fast spin with slow motors.

## Lesson 6 · Build it yourself

Do these in order. Each one is small and teaches one thing.

1. **Wind.** In `quad.step()`, add a constant force, e.g. `+ np.array([0.02, 0, 0])` to the
   acceleration. Does ours still pass the health check? (It drifts: there's no integral term.)
2. **Fix the drift.** In `thrust_vector()`, add an integral of the position error (`KI_POS`).
   This is the "wind estimate" from slide 9, in its simplest form.
3. **Tune.** Change `K_ATT`, `K_RATE` and `INDI_CUTOFF_HZ` in `control.py`. When does it oscillate?
4. **A new plot.** Plot the four `f1..f4` columns of `results/log_ours.csv` yourself with pandas or
   matplotlib. The column names are in the first line of the file.
5. **MPC.** Replace `thrust_vector()`'s PD with an MPC (CasADi) that plans a second ahead with the
   tilt and thrust limits as constraints. That's the next big step of the project.

## Useful facts

- Run tests after every change: `.venv/bin/python -m pytest -q`
- Physics runs at 1000 Hz, control at 500 Hz, 2 ms per control step.
- Units: metres, seconds, newtons, radians inside the code; grams and degrees only in plots.
- `QuadParams` values from the URDF are a published Crazyflie model; `motor_tau` and `yaw_drag`
  are our guesses (see README).
