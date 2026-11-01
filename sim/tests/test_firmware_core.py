"""The C controller in firmware/indi_wls.c must match the Python one it was ported from."""
import ctypes
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from allocation import NORMAL_WEIGHTS, wls_allocate
from control import FAULT_CAP, K_ATT, K_RATE, KD_POS, KP_POS, IndiWlsController, Reference, Sensors
from quad import QuadParams, rotation_from_vector

FIRMWARE = Path(__file__).resolve().parents[2] / "firmware"
P = QuadParams()
DT = 0.002

pytestmark = pytest.mark.skipif(shutil.which("gcc") is None, reason="needs gcc")

F3, F4, F9 = ctypes.c_float * 3, ctypes.c_float * 4, ctypes.c_float * 9


class Params(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in ("mass", "fMax", "armD", "kappa")] + [("inertia", F3)] + [
        (n, ctypes.c_float) for n in ("motorTau", "kpPos", "kdPos", "kAtt", "kRate", "normalTilt", "faultTilt",
                                      "thrustMargin", "faultCap", "yawRampS", "indiCutoffHz")]


class State(ctypes.Structure):
    _fields_ = [("gyroF", F3), ("tauF", F3), ("fEst", F4), ("faultMotor", ctypes.c_int), ("faultTime", ctypes.c_float)]


class CSensors(ctypes.Structure):
    _fields_ = [("pos", F3), ("vel", F3), ("rot", F9), ("gyro", F3)]


class CReference(ctypes.Structure):
    _fields_ = [("pos", F3), ("vel", F3), ("acc", F3), ("yaw", ctypes.c_float)]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    so = tmp_path_factory.mktemp("c") / "indi_wls.so"
    subprocess.run(["gcc", "-O2", "-shared", "-fPIC", "-Wall", "-Werror", "-o", str(so),
                    str(FIRMWARE / "indi_wls.c"), "-lm"], check=True)
    c = ctypes.CDLL(str(so))
    c.indiWlsAllocate.argtypes = [ctypes.POINTER(Params), F4, F4, F4, F4]
    return c


def c_params(lib):
    p = Params()
    lib.indiWlsDefaultParams(ctypes.byref(p))
    return p


def test_defaults_match_python(lib):
    p = c_params(lib)
    assert p.fMax == pytest.approx(P.f_max, rel=1e-6)
    assert p.kappa == pytest.approx(P.kappa, rel=1e-6)
    assert p.faultCap == pytest.approx(FAULT_CAP)
    assert (p.kpPos, p.kdPos, p.kAtt, p.kRate) == pytest.approx((KP_POS, KD_POS, K_ATT, K_RATE))


def test_allocation_matches_scipy(lib):
    p = c_params(lib)
    rng = np.random.default_rng(0)
    for i in range(300):
        hover = P.mass * 9.81
        wrench = np.array([rng.uniform(0.2, 1.8) * hover, *rng.normal(0, 3e-3, 2), rng.normal(0, 3e-4)])
        upper = np.full(4, P.f_max)
        weights = NORMAL_WEIGHTS.copy()
        if i % 2:
            upper[rng.integers(4)] *= FAULT_CAP
            weights[3] *= 1e-3
        out = F4()
        lib.indiWlsAllocate(ctypes.byref(p), F4(*wrench), F4(*upper), F4(*weights), out)
        expected = wls_allocate(wrench, P, upper, weights)
        assert np.allclose(list(out), expected, atol=2e-4 * P.f_max), (wrench, upper, list(out), expected)


def test_controller_matches_python_through_a_fault(lib):
    p = c_params(lib)
    state = State()
    lib.indiWlsReset(ctypes.byref(state), ctypes.byref(p))
    py = IndiWlsController(P, DT)
    rng = np.random.default_rng(1)
    for i in range(600):
        t = i * DT
        if i == 300:
            py.notify_fault(0, t)
            lib.indiWlsNotifyFault(ctypes.byref(state), 0, ctypes.c_float(t))
        spin = [0.0, 0.0, 8.0 if i >= 300 else 0.0]      # the drone spins after the fault
        sens = Sensors(pos=rng.normal(0, 0.1, 3) + [0, 0, 0.8], vel=rng.normal(0, 0.2, 3),
                       rot=rotation_from_vector(rng.normal(0, 0.15, 3)), gyro=rng.normal(0, 0.5, 3) + spin)
        ref = Reference(pos=np.array([0.0, 0.0, 0.8]), vel=np.zeros(3), yaw=0.1)
        expected = py(t, sens, ref)
        cs = CSensors(F3(*sens.pos), F3(*sens.vel), F9(*sens.rot.ravel()), F3(*sens.gyro))
        cr = CReference(F3(*ref.pos), F3(*ref.vel), F3(*ref.acc), ref.yaw)
        out = F4()
        lib.indiWlsUpdate(ctypes.byref(state), ctypes.byref(p), ctypes.c_float(DT), ctypes.c_float(t),
                          ctypes.byref(cs), ctypes.byref(cr), out)
        assert np.allclose(list(out), expected, atol=1e-3 * P.f_max), (i, list(out), expected)
