# Our controller on the real Crazyflie

`indi_wls.c` is the INDI + WLS controller (a C port of `sim/control.py`, checked against it by
`sim/tests/test_firmware_core.py`). `controller_oot.c` plugs it into the firmware's out-of-tree
controller slot. The same two files run in CrazySim (see `sim/LINUX_SETUP.md`) and on the drone.

Tested so far in simulation only. The steps below build it into the official firmware (2026.08),
flash it over the Crazyradio, and test it in stages.

## 1 · One-time setup

```bash
sudo apt install -y gcc-arm-none-eabi make git
# Crazyradio and Crazyflie USB permissions (then log out and back in)
sudo groupadd -f plugdev && sudo usermod -aG plugdev $USER
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-crazyradio.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-crazyflie.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

source ~/cfenv/bin/activate
pip install cfclient                    # GUI client + cfloader (flashing tool)

# Official firmware with our fault switch (skip the clone if ~/crazyflie-firmware exists)
cd ~ && git clone --branch 2026.08 --recurse-submodules https://github.com/bitcraze/crazyflie-firmware.git
cd ~/crazyflie-firmware && git apply ~/UAV_FAULT_TOLERANT_QUADROTOR_CONTROL/firmware/crazyflie-firmware.patch
```

`crazyflie-firmware.patch` adds the fault switch to `stabilizer.c`: parameters `fault.motor` (0 off, 1-4)
and `fault.cap` (thrust left, fraction of full thrust), applied before battery compensation so the cap is
the same thrust at any battery level.

**First, update the drone to the official 2026.08 release** with cfclient (`cfclient`, then
*Connect → Bootloader*, pick the 2026.08 release). That also updates the radio chip, which our build
does not touch. Fly it once with a stock hover from cfclient to know the drone is fine.

## 2 · Build and flash

```bash
cd ~/UAV_FAULT_TOLERANT_QUADROTOR_CONTROL/firmware
make cf2_defconfig      # once: Crazyflie 2.x (brushed) configuration
make -j8                # builds build/cf2.bin with our controller
```
Put the drone in bootloader mode: switch it off, then hold the power button about 2 s until the blue
LEDs blink. Then:
```bash
source ~/cfenv/bin/activate
make cload
```
Check it worked: connect with cfclient, *Parameters* tab: `indiwls.controllerId` should read 6, and the
groups `indiwls` and `fault` should exist. Nothing changes in flight until you select our controller.

## 3 · Test in stages

You need a positioning deck (Flow deck v2, Loco or Lighthouse) for anything below: the script flies to
positions. Weigh the drone as it flies (battery and decks) and pass it as `--mass` in kg.

Every stage: soft floor or a net, nobody within 2 m, a fresh battery, and your hand on the keyboard.
**Ctrl+C in the script stops the motors.** Do the fault tests in the first two minutes of a battery.

```bash
cd ~/UAV_FAULT_TOLERANT_QUADROTOR_CONTROL/sim && source ~/cfenv/bin/activate
R="--uri radio://0/80/2M/E7E7E7E7E7 --model cf2x_P250 --mass 0.030"   # your weight here

python3 crazysim/rectangle_cflib.py $R --controller stock --hover 5 --no-fault   # A: setup works
python3 crazysim/rectangle_cflib.py $R --controller ours  --hover 5 --no-fault   # B: our controller hovers
python3 crazysim/rectangle_cflib.py $R --controller ours  --hover 8 --fault-cap 0.5   # C: mild fault
python3 crazysim/rectangle_cflib.py $R --controller stock --hover 8 --fault-cap 0.5   # D: stock, same fault
python3 crazysim/rectangle_cflib.py $R --controller ours  --hover 4 --fault-cap 0.25 --land-on-fault 1.5   # E: 50% RPM, land
python3 crazysim/rectangle_cflib.py $R --controller stock --hover 4 --fault-cap 0.25 --land-on-fault 1.5   # F: same, stock
python3 crazysim/rectangle_cflib.py $R --controller ours  --no-fault                  # G: rectangle
python3 crazysim/rectangle_cflib.py $R --controller ours  --fault-cap 0.5             # H: rectangle + fault
```
Move to the next stage only when the previous one looks right. If B wobbles or drifts, stop and send
the log (`sim/results/real_*.csv`). Stage D is expected to crash from 0.5 m.

`--fault-cap 0.5` is half the thrust (about 70% RPM). The project's fault, 50% RPM, is `--fault-cap 0.25`.

## What to expect (rehearsed in CrazySim, model `cf2x_P250` = Crazyflie 2.1 with 2.1+ propellers, 27 g)

| Test | Ours | Stock |
|---|---|---|
| hover, no fault | hovers (tilt 0.1 deg) | hovers |
| hover, fault cap 0.5 | stays level (1.3 deg), lands | crashes |
| hover, fault cap 0.25 (50% RPM) | level for ~2 s, then tips over | crashes |
| 50% RPM at 0.5 m, land straight away (`--land-on-fault 1.5`) | **level (1 deg), touches down at 0.6 m/s** | falls, hits at 2.2 m/s |
| rectangle, fault cap 0.5 | passes the health check, then crashes while continuing | crashes |

The firmware rates this drone at 0.12 N per motor. With 50% RPM on one motor that is barely more thrust
than its weight, so the 50% RPM fault is at the physical limit of a brushed Crazyflie 2.1: it can't keep
flying the path, but it can land safely. To keep flying after a 50% RPM fault, the drone needs more thrust:
in CrazySim the Crazyflie 2.1 Brushless (`cf21B_500`) finishes the rectangle (`sim/README.md`). These numbers
assume CrazySim's motor speed (0.13 s) and yaw drag (`--yaw-drag 2e-5`); measure both on the real drone
and set `indiwls.motorTau` (or `MODELS` in `rectangle_cflib.py`).
