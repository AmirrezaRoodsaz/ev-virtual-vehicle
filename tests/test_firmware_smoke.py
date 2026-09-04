"""S1 smoke test: the vendored C builds, loads, and is callable from Python.

Scope is deliberately narrow — the pure transforms and SVPWM, which need no
motor parameters. Their correctness is already proven upstream in
edrive-foc-control; what this test guards is *this* repo's build and ctypes
binding. If a struct layout drifts or the vendoring breaks, these fail.
"""

import ctypes
import math

import numpy as np
import pytest

from sim import clib


@pytest.fixture(scope="module")
def lib():
    return clib.load()


def test_library_loads(lib):
    assert lib is not None


def test_clarke_balanced_currents(lib):
    """A balanced three-phase set maps to a rotating vector of the same peak.

    Amplitude-invariant Clarke: |i_alphabeta| equals the peak phase current.
    """
    peak = 100.0
    for theta in np.linspace(0.0, 2 * math.pi, 13):
        ia = peak * math.cos(theta)
        ib = peak * math.cos(theta - 2 * math.pi / 3)
        ic = peak * math.cos(theta + 2 * math.pi / 3)
        al, be, pal, pbe = clib.out2()
        lib.foc_clarke(ia, ib, ic, pal, pbe)
        assert math.hypot(al.value, be.value) == pytest.approx(peak, rel=1e-4)


def test_park_inverse_park_round_trip(lib):
    """Park then inverse-Park is the identity, at every rotor angle."""
    for theta in np.linspace(0.0, 2 * math.pi, 17):
        s, c = math.sin(theta), math.cos(theta)
        d_in, q_in = 12.5, -47.25

        al, be, pal, pbe = clib.out2()
        lib.foc_inv_park(d_in, q_in, s, c, pal, pbe)

        d, q, pd, pq = clib.out2()
        lib.foc_park(al.value, be.value, s, c, pd, pq)

        assert d.value == pytest.approx(d_in, abs=1e-3)
        assert q.value == pytest.approx(q_in, abs=1e-3)


def test_svpwm_duties_are_bounded_and_balanced(lib):
    """Duties stay inside [0,1] and their differences track the phase voltages.

    Min-max common-mode injection shifts all three duties together, so the
    *differences* carry the line-to-line voltage while the common mode is free.
    """
    v_dc = 325.0
    v_mag = v_dc / math.sqrt(3.0) * 0.95  # just inside the linear region
    for theta in np.linspace(0.0, 2 * math.pi, 25):
        va, vb = v_mag * math.cos(theta), v_mag * math.sin(theta)
        da, db, dc = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
        lib.foc_svpwm(va, vb, v_dc,
                      ctypes.byref(da), ctypes.byref(db), ctypes.byref(dc))
        for d in (da.value, db.value, dc.value):
            assert 0.0 <= d <= 1.0

        # phase a minus phase b duty -> line-to-line voltage v_ab
        v_ab_expected = va - (-0.5 * va + math.sqrt(3.0) / 2.0 * vb)
        assert (da.value - db.value) * v_dc == pytest.approx(v_ab_expected, rel=2e-3)


def test_pi_anti_windup_holds_integrator_at_the_limit(lib):
    """Driving a PI hard into its output limit must not wind the integrator up.

    Conditional integration: once clamped, further same-sign error is ignored,
    so the controller leaves the limit as soon as the error reverses.
    """
    pi = clib.FocPi(kp=1.0, ki=100.0, ts=1e-4, integ=0.0)
    for _ in range(1000):
        lib.foc_pi_step(ctypes.byref(pi), 50.0, -10.0, 10.0)
    wound = pi.integ

    # Unbounded integration of err*ki*ts over 1000 steps would reach 500.
    assert wound < 20.0, "integrator wound up past the output limit"

    out = lib.foc_pi_step(ctypes.byref(pi), -50.0, -10.0, 10.0)
    assert out < 10.0, "controller did not leave the limit on error reversal"
