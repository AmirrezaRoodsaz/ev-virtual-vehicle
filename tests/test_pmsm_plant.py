"""S2: validate the C PMSM plant against analytic solutions.

The brief's rule is that physics is validated before any controller or
estimator is built on top of it — otherwise a later tuning bug and a modelling
bug are indistinguishable. Every test here compares the C against a closed-form
result, not against a second implementation of the same equations.

Two fixtures recur:

* **Clamped rotor** — inertia set enormous so mechanical speed is frozen at its
  initial value. This isolates the electrical subsystem so its response can be
  compared to the first-order analytic solution. It is a test fixture, not a
  physical claim.
* **Free rotor** — the real inertia, used to check the mechanical ODE.
"""

import ctypes
import math

import numpy as np
import pytest

from sim import clib
from sim.params import MOTOR


@pytest.fixture(scope="module")
def lib():
    return clib.load()


@pytest.fixture
def par():
    return clib.motor_params()


@pytest.fixture
def clamped():
    """Rotor frozen: inertia large enough that torque cannot move it."""
    p = clib.motor_params()
    p.j = 1e12
    p.b = 0.0
    return p


def steady_currents(vd, vq, w_e, m=MOTOR):
    """Analytic (id, iq) solving d/dt = 0 at fixed vd, vq, w_e."""
    a = np.array([[m.Rs, -w_e * m.Lq], [w_e * m.Ld, m.Rs]])
    b = np.array([vd, vq - w_e * m.psi_f])
    return np.linalg.solve(a, b)


def steady_voltages(i_d, i_q, w_e, m=MOTOR):
    """Analytic inverse: the (vd, vq) that hold (id, iq) at fixed w_e."""
    return (m.Rs * i_d - w_e * m.Lq * i_q,
            m.Rs * i_q + w_e * (m.Ld * i_d + m.psi_f))


# ---------------------------------------------------------------- torque

def test_torque_matches_analytic(lib, par):
    """Te = 1.5 p (psi_f iq + (Ld - Lq) id iq), reluctance term included."""
    for i_d, i_q in [(0.0, 0.0), (0.0, 300.0), (-150.0, 400.0),
                     (100.0, -200.0), (-50.0, 50.0)]:
        expect = 1.5 * MOTOR.p * (MOTOR.psi_f * i_q
                                  + (MOTOR.Ld - MOTOR.Lq) * i_d * i_q)
        got = lib.pmsm_torque(ctypes.byref(par), i_d, i_q)
        assert got == pytest.approx(expect, rel=1e-12, abs=1e-12)


def test_reluctance_torque_has_the_right_sign(lib, par):
    """Ld < Lq for an IPM, so negative id adds torque — the basis of MTPA.

    Not used by this project (MTPA is out of scope), but if this sign were
    wrong the plant would be modelling a machine that does not exist.
    """
    t_zero_d = lib.pmsm_torque(ctypes.byref(par), 0.0, 300.0)
    t_neg_d = lib.pmsm_torque(ctypes.byref(par), -100.0, 300.0)
    assert MOTOR.Ld < MOTOR.Lq
    assert t_neg_d > t_zero_d


# ------------------------------------------------------------ electrical

def test_at_rest_with_no_excitation_nothing_moves(lib, par):
    x = clib.PmsmState()
    lib.pmsm_init(ctypes.byref(x))
    for _ in range(1000):
        lib.pmsm_step(ctypes.byref(par), ctypes.byref(x), 0.0, 0.0, 0.0, 1e-5)
    assert x.i_d == 0.0 and x.i_q == 0.0
    assert x.w_m == 0.0 and x.theta_m == 0.0


def test_q_axis_step_response_matches_first_order_analytic(lib, clamped):
    """At standstill the axes decouple: iq(t) = (vq/Rs)(1 - exp(-t Rs/Lq)).

    Checked along the whole trajectory, not just the final value, so an
    integration error shows up as a timing error rather than hiding in the
    steady state.
    """
    vq, dt, n = 5.0, 1e-6, 3000
    x = clib.PmsmState()
    lib.pmsm_init(ctypes.byref(x))

    tau = MOTOR.Lq / MOTOR.Rs
    final = vq / MOTOR.Rs
    for k in range(1, n + 1):
        lib.pmsm_step(ctypes.byref(clamped), ctypes.byref(x), 0.0, vq, 0.0, dt)
        t = k * dt
        expect = final * (1.0 - math.exp(-t / tau))
        assert x.i_q == pytest.approx(expect, rel=1e-6, abs=1e-9)
        assert x.i_d == pytest.approx(0.0, abs=1e-12)


def test_steady_state_currents_match_analytic_under_rotation(lib, clamped):
    """Spinning, the axes cross-couple. Drive the plant with the voltages
    that analytically hold a target (id, iq) and confirm it settles there."""
    w_m = 300.0                      # ~2900 rpm mechanical
    w_e = MOTOR.p * w_m
    target_d, target_q = -80.0, 250.0
    vd, vq = steady_voltages(target_d, target_q, w_e)

    x = clib.PmsmState()
    lib.pmsm_init(ctypes.byref(x))
    x.w_m = w_m

    # The electrical time constant is Lq/Rs ~ 29.5 ms, so settling needs
    # hundreds of milliseconds: at 0.2 s the residual exp(-6.8) ~ 1e-3 is
    # still larger than the tolerance below. 1.0 s is ~34 tau. dt = 10 us is
    # far inside both that time constant and the 5.2 ms rotation period.
    dt = 1e-5
    for _ in range(100_000):         # 1.0 s
        lib.pmsm_step(ctypes.byref(clamped), ctypes.byref(x), vd, vq, 0.0, dt)

    assert x.w_m == pytest.approx(w_m, rel=1e-9)   # clamp really held
    assert x.i_d == pytest.approx(target_d, rel=1e-6)
    assert x.i_q == pytest.approx(target_q, rel=1e-6)


def test_forward_and_inverse_steady_state_agree(lib, clamped):
    """The two analytic helpers are inverses — guards the test oracle itself."""
    w_e = MOTOR.p * 250.0
    for i_d, i_q in [(0.0, 100.0), (-120.0, 380.0), (60.0, -90.0)]:
        vd, vq = steady_voltages(i_d, i_q, w_e)
        back_d, back_q = steady_currents(vd, vq, w_e)
        assert back_d == pytest.approx(i_d, abs=1e-9)
        assert back_q == pytest.approx(i_q, abs=1e-9)


# ------------------------------------------------------------ mechanical

def test_mechanical_ode_is_integrated_correctly(lib, par):
    """One small step must satisfy J dw/dt = Te - T_load - B w.

    Compared against the plant's own torque so this isolates the mechanical
    integration; the torque expression is verified separately above.
    """
    x = clib.PmsmState()
    lib.pmsm_init(ctypes.byref(x))
    x.i_q, x.w_m = 300.0, 50.0
    t_load, dt = 40.0, 1e-7

    te = lib.pmsm_torque(ctypes.byref(par), x.i_d, x.i_q)
    expect_dw = (te - t_load - MOTOR.B * x.w_m) / MOTOR.J * dt
    w0 = x.w_m

    vd, vq = steady_voltages(x.i_d, x.i_q, MOTOR.p * x.w_m)
    lib.pmsm_step(ctypes.byref(par), ctypes.byref(x), vd, vq, t_load, dt)

    assert (x.w_m - w0) == pytest.approx(expect_dw, rel=1e-6)


def test_load_torque_decelerates_a_spinning_unexcited_rotor(lib, par):
    x = clib.PmsmState()
    lib.pmsm_init(ctypes.byref(x))
    x.w_m = 200.0
    for _ in range(10_000):
        lib.pmsm_step(ctypes.byref(par), ctypes.byref(x), 0.0, 0.0, 20.0, 1e-5)
    assert x.w_m < 200.0


# ----------------------------------------------------------------- angle

def test_electrical_angle_wraps_and_tracks_pole_pairs(lib, par):
    x = clib.PmsmState()
    lib.pmsm_init(ctypes.byref(x))
    x.w_m = 400.0

    for _ in range(50_000):
        lib.pmsm_step(ctypes.byref(par), ctypes.byref(x), 0.0, 0.0, 0.0, 1e-5)
        th_e = lib.pmsm_theta_e(ctypes.byref(par), ctypes.byref(x))
        assert 0.0 <= th_e < 2.0 * math.pi
        assert 0.0 <= x.theta_m < 2.0 * math.pi

    assert lib.pmsm_w_e(ctypes.byref(par), ctypes.byref(x)) == pytest.approx(
        MOTOR.p * x.w_m, rel=1e-12)


# ------------------------------------------- inverter and the sensor view

def test_inverter_recovers_the_voltage_the_controller_asked_for(lib):
    """vd,vq -> inverse Park -> SVPWM duties -> back to vd,vq.

    Crosses both C modules: the vendored float controller path and the new
    double plant path. Tolerance is loose because the controller side is
    single precision by design.
    """
    v_dc = MOTOR.v_dc
    v_mag = v_dc / math.sqrt(3.0) * 0.9
    for theta_e in np.linspace(0.0, 2 * math.pi, 19):
        vd_ref, vq_ref = 0.3 * v_mag, 0.8 * v_mag
        s, c = math.sin(theta_e), math.cos(theta_e)

        al, be, pal, pbe = clib.out2()
        lib.foc_inv_park(vd_ref, vq_ref, s, c, pal, pbe)

        da, db, dc = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
        lib.foc_svpwm(al.value, be.value, v_dc,
                      ctypes.byref(da), ctypes.byref(db), ctypes.byref(dc))

        duty = (ctypes.c_double * 3)(da.value, db.value, dc.value)
        vals, ptrs = clib.dout(2)
        lib.pmsm_duties_to_vdq(ctypes.byref(duty), v_dc, theta_e, *ptrs)

        assert vals[0].value == pytest.approx(vd_ref, rel=1e-3, abs=1e-2)
        assert vals[1].value == pytest.approx(vq_ref, rel=1e-3, abs=1e-2)


def test_sensor_view_round_trips_through_the_controller_transforms(lib):
    """dq -> phase currents (plant) -> Clarke+Park (controller) -> dq."""
    for theta_e in np.linspace(0.0, 2 * math.pi, 19):
        i_d, i_q = -95.0, 310.0
        vals, ptrs = clib.dout(3)
        lib.pmsm_dq_to_phase(i_d, i_q, theta_e, *ptrs)
        ia, ib, ic = (v.value for v in vals)

        assert ia + ib + ic == pytest.approx(0.0, abs=1e-9)  # floating neutral

        s, c = math.sin(theta_e), math.cos(theta_e)
        al, be, pal, pbe = clib.out2()
        lib.foc_clarke(ia, ib, ic, pal, pbe)
        d, q, pd, pq = clib.out2()
        lib.foc_park(al.value, be.value, s, c, pd, pq)

        assert d.value == pytest.approx(i_d, rel=1e-4, abs=1e-2)
        assert q.value == pytest.approx(i_q, rel=1e-4, abs=1e-2)
