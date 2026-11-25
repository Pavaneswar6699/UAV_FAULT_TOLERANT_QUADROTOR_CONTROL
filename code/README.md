# Team LUGER – Project 6: Fault-tolerant control on the Crazyflie 2.1

One motor is limited to 50% RPM mid-mission (about 25% thrust). Our controller is told which
motor, **drops yaw** (the weak motor's diagonal partner slows to match it, so the drone stays
level while spinning) and **lands** under control. MPC (laptop) plans the path; INDI (drone)
controls the attitude.

```
laptop (Python)                                   Crazyflie (C, firmware app)
mission -> MPC (thrust limits) --radio-->  position tracking -> tilt control -> INDI rate loop
mocap -> extpos ---------------radio-->      -> fault-aware mixer (drops yaw) -> 4 motor forces
```

| Folder | What it is |
|---|---|
| `ftqc/` | Python: parameters, mixer, controller (twin of the C code), MPC, mission, simulator |
| `scripts/run_sim.py` | Simulates healthy / PID baseline with fault / our controller with fault |
| `scripts/hw_calibrate_motor.py` | Bench tool: tachometer readings -> command for 50% RPM |
| `scripts/hw_fly_mission.py` | Real flight: mocap bridge, MPC loop, fault trigger, CSV log |
| `scripts/plot_log.py` | Plots a real-flight log |
| `firmware/app_ftc/` | Crazyflie out-of-tree controller app (`stabilizer.controller = 6`) |
| `firmware/test_host/` | PC test for the C controller core |

## 1. Simulation (any laptop, start here)

```bash
cd code
python -m pip install numpy scipy matplotlib
python scripts/run_sim.py                     # results/*.png, *.csv, summary.json
python scripts/run_sim.py --notify-delay 0.1  # fault notification arrives 100 ms late
python scripts/run_sim.py --mass 0.029        # use your measured mass (kg)
python -m unittest discover -s tests -v       # tests
```

Expected (31 g with markers): `baseline` lands, `pid` tips to ~70° and crashes, `ftc` stays
within ~5° tilt while spinning ~24 rad/s and lands upright.

**Margin warning:** with stock motors (0.12 N each), after the fault the drone can only just
hold its weight. At 31 g it sinks slowly (touchdown ~0.52 m/s); at 29 g ~0.15 m/s. Keep the
markers and mount as light as possible and weigh the drone.

**Mass must match in two places.** After weighing (markers on), set `QuadParams.mass` in
`ftqc/params.py` (kg) and `CONFIG_MODIFIED_CF_MASS` in `firmware/app_ftc/app-config` (milligrams),
then rebuild the firmware. `tests/test_firmware_config.py` fails if they disagree. With the
firmware's default 29 g on a 31 g drone, the simulation shows it flying ~5 cm low and touching
down after the fault at ~0.74 m/s instead of ~0.52 m/s.

C core test (needs gcc):
```bash
cd firmware/test_host
gcc -std=c99 -O2 -I../app_ftc/src test_ftc_core.c ../app_ftc/src/ftc_core.c -lm -o test_ftc_core
./test_ftc_core
```

## 2. Build and flash the firmware (Ubuntu or WSL)

```bash
sudo apt install git make gcc-arm-none-eabi python3-pip
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git ~/crazyflie-firmware
pip install cflib                                  # provides the radio flasher
cd code/firmware/app_ftc
make CRAZYFLIE_BASE=~/crazyflie-firmware -j8
# bootloader: with the drone off, hold the power button ~3 s until the blue LEDs blink
make CRAZYFLIE_BASE=~/crazyflie-firmware cload
```

**On Windows with WSL:** `make` breaks on paths with spaces, and WSL can't see the Crazyradio.
Copy the app into WSL, build there, then flash from Windows PowerShell:

```bash
# in WSL (Ubuntu)
cp -r "/mnt/c/Users/Venkata Shashank/OneDrive/Documents/SEM 5 courses/UAV/Project/code/firmware/app_ftc" ~/app_ftc
cd ~/app_ftc && make -j8
cp build/cf2.bin /mnt/c/Users/Public/cf2_ftc.bin
```
```powershell
# in Windows PowerShell (drone on, Crazyradio plugged in)
cfloader flash C:\Users\Public\cf2_ftc.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E7
```

If your drone is a plain Crazyflie 2.1 with the original propellers (not 2.1+), add
`CONFIG_CRAZYFLIE_LEGACY_PROPELLERS=y` to `app-config` before building. cfclient shows the model.

Parameters (group `ftc`, editable from cfclient or Python): `faultMotor` (fault notification,
0..4), `faultCap` (0.25), `injMotor`/`injCap` (test-only emulated limit), gains `kpXY kdXY kpZ kdZ
kAtt kRate leadTime`. Logs: `ftc.f1..f4`, `ftc.thrust`, `ftc.mode`.

## 3. Bench calibration of the 50% RPM limit (props on, drone clamped down)

```bash
python -m pip install cflib
python scripts/hw_calibrate_motor.py --motor 1
```

Type the tachometer reading at each step. Then do a tethered hover with `ftc.injMotor=1` and
adjust `ftc.injCap` until the tachometer shows 50% of max RPM on that motor.

## 4. Real flights (with the lab motion capture)

1. In the mocap software, create a rigid body named `cf` from 3–4 markers in an **uneven**
   pattern, with x pointing to the drone's front.
2. `python -m pip install -r requirements.txt`
3. Healthy baseline with the stock controller (no custom firmware needed):
   `python scripts/hw_fly_mission.py --controller pid --no-fault --mocap-type optitrack --mocap-host <PC IP>`
4. Our controller, healthy: `python scripts/hw_fly_mission.py --controller ftc --no-fault ...`
5. Fault test, **tethered and low first**:
   `python scripts/hw_fly_mission.py --controller ftc --fault-motor 1 --fault-waypoint 2 ...`
6. Demo day if the evaluator applies the limit: add `--fault-key --no-inject` and press `f` when told.

Keys: `f` = trigger the fault, `q` or Ctrl+C = **kill motors**. Logs: `logs/flight_*.csv`,
plot with `python scripts/plot_log.py logs/flight_....csv`.

## Before the first real flight

- The flight scripts and firmware app are written against the current Bitcraze firmware and
  cflib, but have not been flown yet. Fly the healthy `ftc` mission before any fault test.
- Gains in `ftqc/params.py` and the firmware defaults are the simulation-tuned values. Tune on
  the real drone through the `ftc.*` parameters (start with `kRate`, `kAtt`, then `leadTime`).
- Check the spin rate in the logs: the gyro saturates at 2000 deg/s (~35 rad/s); the simulation
  predicts ~24 rad/s.
