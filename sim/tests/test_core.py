import numpy as np
import pytest

from allocation import FAULT_YAW_WEIGHT, NORMAL_WEIGHTS, stock_mix, wls_allocate
from control import FAULT_CAP
from quad import GRAVITY, QuadParams, allocation_matrix, hover_state, step

P = QuadParams()
HOVER = np.array([P.mass * GRAVITY, 0.0, 0.0, 0.0])
FAULT_WEIGHTS = NORMAL_WEIGHTS * np.array([1, 1, 1, FAULT_YAW_WEIGHT])


def test_allocation_matrix_inverse_matches_crazyflie_mixer():
    d = P.arm / np.sqrt(2)
    first_motor = np.linalg.inv(allocation_matrix(P))[0]
    expected = [0.25, -0.25 / d, -0.25 / d, -0.25 / P.kappa]
    assert first_motor == pytest.approx(expected)


def test_hover_thrust_holds_position():
    state = hover_state(P, [0, 0, 1])
    for _ in range(1000):
        state = step(state, state.thrust, np.full(4, P.f_max), P, 0.001)
    assert np.linalg.norm(state.pos - [0, 0, 1]) < 1e-6


def test_wls_matches_plain_inverse_when_nothing_saturates():
    wrench = HOVER + np.array([0.0, 1e-4, -1e-4, 0.0])
    expected = np.linalg.solve(allocation_matrix(P), wrench)
    assert wls_allocate(wrench, P, np.full(4, P.f_max), NORMAL_WEIGHTS) == pytest.approx(expected, abs=2e-4)


def test_wls_fault_split_matches_slide_numbers():
    upper = np.array([FAULT_CAP, 1, 1, 1]) * P.f_max
    f = wls_allocate(HOVER, P, upper, FAULT_WEIGHTS) / GRAVITY * 1000   # grams
    weak, partner, others = f[0], f[2], f[[1, 3]]
    assert weak == pytest.approx(FAULT_CAP * P.f_max / GRAVITY * 1000, abs=0.1)
    assert partner == pytest.approx(weak, abs=0.1)                     # diagonal matches -> level
    assert others == pytest.approx(np.full(2, (27.0 - 2 * weak) / 2), abs=0.1)


def test_stock_mix_lowers_all_motors_equally_when_saturated():
    f = stock_mix(np.array([4 * P.f_max, 2e-3, 0.0, 0.0]), P)
    assert f.max() == pytest.approx(P.f_max)
    unsaturated = np.linalg.solve(allocation_matrix(P), [4 * P.f_max, 2e-3, 0.0, 0.0])
    assert np.diff(unsaturated - f) == pytest.approx(np.zeros(3), abs=1e-9)
