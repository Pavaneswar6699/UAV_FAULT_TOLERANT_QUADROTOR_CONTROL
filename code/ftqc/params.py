"""Physical parameters and default gains for the brushed Crazyflie 2.1.

The constants come from crazyflie-firmware `platform_defaults_cf2.h`
(ARM_LENGTH, CF_MASS, THRUST_MAX, THRUST2TORQUE). Re-measure the mass with
your mocap markers on and pass it in, because the post-fault margin is tiny.
"""
from dataclasses import dataclass, field

import numpy as np

G = 9.81


@dataclass
class QuadParams:
    mass: float = 0.031          # kg: 29 g stock + ~2 g of mocap markers
    arm: float = 0.046           # m, motor to centre (ARM_LENGTH)
    f_max: float = 0.12          # N per motor (THRUST_MAX, standard brushed motors)
    k_torque: float = 0.0069929  # m, yaw torque per newton of thrust (THRUST2TORQUE)
    inertia: np.ndarray = field(
        default_factory=lambda: np.diag([1.66e-5, 1.66e-5, 2.93e-5]))  # kg m^2
    tau_motor: float = 0.03      # s, first-order motor + rotor lag
    yaw_drag: float = 2.0e-6     # N m s^2, quadratic drag that limits the post-fault spin
    lin_drag: float = 0.01       # N s/m

    @property
    def weight(self) -> float:
        return self.mass * G


@dataclass
class Gains:
    # onboard position tracking around the MPC reference (P on position, D on velocity)
    kp_xy: float = 4.0
    kd_xy: float = 3.0
    kp_z: float = 6.0
    kd_z: float = 4.0
    tilt_max: float = np.radians(25.0)
    # attitude: thrust-direction error -> body-rate command (1/s)
    k_att: float = 8.0
    k_yaw: float = 2.0
    # INDI rate loop: rate error -> angular-acceleration command (1/s)
    k_rate: float = 25.0
    lpf_hz: float = 30.0          # 2nd-order filter on gyro and on the actuator estimate
    lead_time: float = 0.035      # s, spin compensation after the fault (motor lag + filter delay)
    dt: float = 0.002             # 500 Hz, the firmware ATTITUDE_RATE
