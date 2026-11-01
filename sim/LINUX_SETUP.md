# Working on Linux (Ubuntu)

Use **Ubuntu 22.04 LTS** (24.04 also works for everything here except Gazebo; we use MuJoCo instead).

## 1 · Move the project over

Copy the `UAV_Project` folder (USB, Google Drive, or a GitHub repo). **Don't copy `sim/.venv`**:
it's built for the Mac. Recreate it in step 2.

## 2 · Our Python simulator (≈5 minutes)

```bash
sudo apt update && sudo apt install -y git python3-venv python3-pip python3-tk
cd ~/UAV_Project/sim
python3 -m venv .venv
.venv/bin/pip install numpy scipy matplotlib pytest
.venv/bin/python -m pytest -q          # expect: 10 passed
.venv/bin/python run_sim.py            # stock crashes, ours finishes
.venv/bin/python export_viewer.py && xdg-open results/flight_viewer.html
```
`python3-tk` lets `animate.py` open its window.

## 3 · CrazySim: the real Crazyflie firmware in a simulator (≈1 hour)

CrazySim runs the actual firmware on your laptop and shows the drone in a MuJoCo 3D window.
Your laptop talks to it with cflib exactly as it would talk to the real drone.

```bash
sudo apt install -y cmake build-essential
cd ~
git clone https://github.com/gtfactslab/CrazySim.git --recursive
python3 -m venv ~/cfenv && source ~/cfenv/bin/activate
pip install Jinja2 mujoco numpy

cd ~/CrazySim/crazyflie-firmware
git apply ~/UAV_FAULT_TOLERANT_QUADROTOR_CONTROL/sim/crazysim/firmware.patch
git -C tools/crazyflie-simulation apply ~/UAV_FAULT_TOLERANT_QUADROTOR_CONTROL/sim/crazysim/simulator.patch
mkdir -p sitl_make/build && cd sitl_make/build
cmake .. && make all
cd ~/CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/mujoco/launch/sitl_singleagent.sh -m cf2x_L250 -x 0 -y 0
```
`firmware.patch`:
- **Fixes a hang.** Without it the firmware can stop at `STAB: Wait for sensor calibration...` and cflib
  never connects: `lastWakeTime` is `uint32_t` but the Linux FreeRTOS tick is 64-bit, so the stabilizer
  never sleeps and starves every other task.
- **Adds the fault switch** for milestone 4: parameters `fault.motor` (0 off, 1-4) and `fault.cap`
  (thrust left, default 0.25), and the log group `fault.m1`-`fault.m4` (PWM actually sent to the motors).
- **Builds our controller in** (`../firmware/*.c`, from this repo's folder in your home directory; set
  `cmake -DINDI_WLS_DIR=...` if the repo is elsewhere) as controller 5, and lets the firmware send
  per-motor forces to CrazySim.

`simulator.patch`:
- adds `--yaw-drag <k>`: air drag against spinning, which CrazySim leaves out. Without it a drone that
  loses a motor spins faster and faster until the gyro saturates.
- keeps the simulation at real time (RTF 1.00 in the bottom-right corner of the 3D window). The firmware
  and the flight script run on the wall clock, so a slower simulation flies the path faster than
  planned and shortens the hold after the fault: at RTF 0.8 our controller failed the health check.

`make all` also prints a `gz-plugin` CMake error: that's the Gazebo version of CrazySim, ignore it.
The firmware is built once `Built target cf2` appears.

Models: `cf2x_L250` = standard Crazyflie 2.x, `cf2x_T350` = thrust upgrade kit, `cf21B_500` = 2.1 Brushless.
Pick the one matching the drone the lab gives you.

## 4 · cflib from source (CrazySim needs the newest version)

In a second terminal:
```bash
source ~/cfenv/bin/activate
cd ~ && git clone https://github.com/bitcraze/crazyflie-lib-python.git
cd crazyflie-lib-python && pip install -e .
cd ~/UAV_FAULT_TOLERANT_QUADROTOR_CONTROL/sim && python3 crazysim/rectangle_cflib.py --controller stock --no-fault
```
The drone in the MuJoCo window should take off, fly A → B → C → D → A and land. That's your
**baseline flight** (D2, D3) running on the real firmware.

Without `--no-fault` the script caps M1 at 25% thrust midway along B → C (`--fault-motor`,
`--fault-cap`, `--fault-delay` change that), then holds position for 2.5 s and runs the same health
check as `run_sim.py`: continue at half speed, or land where it is. Each run saves a 50 Hz log in
`results/crazysim_*.csv`. After a crash, restart CrazySim (Ctrl+C in its terminal, launch again).

**Our controller vs the stock one, with the fault.** Start CrazySim with a drone that has the thrust
to survive it and with yaw drag:
```bash
# terminal 1
bash tools/crazyflie-simulation/simulator_files/mujoco/launch/sitl_singleagent.sh -m cf21B_500 -x 0 -y 0 --yaw-drag 2e-5
# terminal 2
python3 crazysim/rectangle_cflib.py --model cf21B_500 --controller ours
python3 crazysim/rectangle_cflib.py --model cf21B_500 --controller stock   # restart CrazySim first
```
`--model` must match `-m`: the script sets our controller's mass, motor strength and motor speed from
it. Results for every model are in `README.md`.

If it complains about the URI or arming, check CrazySim's README: it changes faster than this file.

## Milestones on Linux

| # | Goal | Deliverable |
|---|---|---|
| 1 | Our Python sim runs | — |
| 2 | CrazySim drone hovers from cflib | — |
| 3 | Rectangle flown in CrazySim with the stock controller | D2, D3 (in simulation) |
| 4 | Firmware fault switch: a parameter that caps motor k at 50% RPM; watch the stock controller crash | D4 |
| 5 | Our controller (INDI + WLS) in the firmware; fault flight in CrazySim | D5, D6 |

Milestones 1-5 run in simulation. On the real drone: [`../firmware/README.md`](../firmware/README.md).
