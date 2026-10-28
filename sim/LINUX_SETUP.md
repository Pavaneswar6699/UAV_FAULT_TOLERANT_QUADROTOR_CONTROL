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
.venv/bin/python -m pytest -q          # expect: 5 passed
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
git submodule update --init tools/crazyflie-simulation/simulator_files/mujoco/drone-models
mkdir -p sitl_make/build && cd sitl_make/build
cmake .. && make all
cd ~/CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/mujoco/launch/sitl_singleagent.sh -m cf2x_L250 -x 0 -y 0
```
Models: `cf2x_L250` = standard Crazyflie 2.x, `cf2x_T350` = thrust upgrade kit, `cf21B_500` = 2.1 Brushless.
Pick the one matching the drone the lab gives you.

## 4 · cflib from source (CrazySim needs the newest version)

In a second terminal:
```bash
source ~/cfenv/bin/activate
cd ~ && git clone https://github.com/bitcraze/crazyflie-lib-python.git
cd crazyflie-lib-python && pip install -e .
cd ~/UAV_Project/sim && python3 crazysim/rectangle_cflib.py
```
The drone in the MuJoCo window should take off, fly A → B → C → D → A and land. That's your
**baseline flight** (D2, D3) running on the real firmware.

If it complains about the URI or arming, check CrazySim's README: it changes faster than this file.

## Milestones on Linux

| # | Goal | Deliverable |
|---|---|---|
| 1 | Our Python sim runs | — |
| 2 | CrazySim drone hovers from cflib | — |
| 3 | Rectangle flown in CrazySim with the stock controller | D2, D3 (in simulation) |
| 4 | Firmware fault switch: a parameter that caps motor k at 50% RPM; watch the stock controller crash | D4 |
| 5 | Our controller (INDI + WLS) as a firmware app; fault flight in CrazySim | D5, D6 |

Steps 4 and 5 are firmware work (C). Ask for help when you reach them.
