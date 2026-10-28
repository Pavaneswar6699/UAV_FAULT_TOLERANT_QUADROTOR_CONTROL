"""Fly the demo rectangle on a simulated Crazyflie in CrazySim (stock firmware controller).

Start CrazySim first (see LINUX_SETUP.md), then:
  python3 crazysim/rectangle_cflib.py
This is the baseline flight (no fault). Not yet tested here: CrazySim needs Linux.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "udp://127.0.0.1:19850"     # CrazySim's first drone
HEIGHT = 0.8                      # [m]
SIDE = 1.5                        # [m]
LEG_TIME = 5.0                    # seconds per side (0.3 m/s)
CORNERS = [(SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE), (0.0, 0.0)]   # A -> B -> C -> D -> A


def fly(cf: Crazyflie) -> None:
    hl = cf.high_level_commander
    hl.takeoff(HEIGHT, 2.0)
    time.sleep(3.0)
    for x, y in CORNERS:
        print(f"going to ({x:.1f}, {y:.1f})")
        hl.go_to(x, y, HEIGHT, 0.0, LEG_TIME)
        time.sleep(LEG_TIME + 0.5)
    hl.land(0.0, 2.5)
    time.sleep(3.0)
    hl.stop()


def main() -> None:
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        scf.cf.platform.send_arming_request(True)   # recent firmware needs arming before flight
        time.sleep(1.0)
        fly(scf.cf)


if __name__ == "__main__":
    main()
