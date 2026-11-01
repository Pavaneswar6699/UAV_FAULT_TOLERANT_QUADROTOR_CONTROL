import numpy as np
import pytest

from control import IndiWlsController, Reference, Sensors
from quad import QuadParams, hover_state, step
from wind_kf import initial_estimate, predict, thrust_acceleration, update

P = QuadParams()
DT = 0.002
WIND = np.array([2.0, -1.0, 0.0])                 # m/s
WIND_ACC = P.lin_drag * WIND / P.mass             # what the filter should find [m/s^2]


def drift_at_hover_thrust(wind, seconds):
    """Level drone at hover thrust drifting in the wind; the filter watches noisy pos/vel."""
    rng = np.random.default_rng(0)
    state = hover_state(P, [0, 0, 1])
    est = initial_estimate(state.pos, state.vel)
    for _ in range(int(seconds / DT)):
        est = predict(est, thrust_acceleration(state.rot, state.thrust, P), P, DT)
        est = update(est, state.pos + rng.normal(0, 0.003, 3), state.vel + rng.normal(0, 0.01, 3))
        state = step(state, state.thrust, np.full(4, P.f_max), P, DT, wind)
    return state, est


def test_wind_pushes_the_drone_downwind():
    state, _ = drift_at_hover_thrust(WIND, 1.0)
    assert state.vel[0] > 0 and state.vel[1] < 0


def test_estimates_constant_wind():
    _, est = drift_at_hover_thrust(WIND, 3.0)
    assert est.wind_acc == pytest.approx(WIND_ACC, abs=0.05)


def test_estimate_stays_near_zero_without_wind():
    _, est = drift_at_hover_thrust(np.zeros(3), 3.0)
    assert np.linalg.norm(est.wind_acc) < 0.05


def test_thrust_acceleration_is_zero_at_hover():
    state = hover_state(P, [0, 0, 1])
    assert thrust_acceleration(state.rot, state.thrust, P) == pytest.approx(np.zeros(3))


def hold_position_error(use_estimate, seconds=6.0):
    """Our controller holds (0, 0, 1) in wind; returns the final position error [m]."""
    ctrl = IndiWlsController(P, DT)
    state = hover_state(P, [0, 0, 1])
    est = initial_estimate(state.pos, state.vel)
    target = np.array([0.0, 0.0, 1.0])
    for _ in range(int(seconds / DT)):
        sens = Sensors(state.pos, state.vel, state.rot, state.omega)
        acc_ff = -est.wind_acc if use_estimate else np.zeros(3)
        f_cmd = ctrl(0.0, sens, Reference(target, np.zeros(3), acc=acc_ff))
        est = predict(est, thrust_acceleration(state.rot, np.clip(f_cmd, 0, P.f_max), P), P, DT)
        est = update(est, state.pos, state.vel)
        state = step(state, f_cmd, np.full(4, P.f_max), P, DT, WIND)
    return np.linalg.norm(state.pos - target)


def test_feeding_the_estimate_to_the_controller_removes_wind_offset():
    assert hold_position_error(False) > 0.05      # PD alone sits downwind
    assert hold_position_error(True) < 0.02
