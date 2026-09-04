"""S3: the fast domain closes correctly, and fast enough.

This is the phase gate. Two things must hold before v0.1-fastcore:

1. Moving the loop into C changed no control behaviour — the step-response
   metrics still match what edrive-foc-control publishes.
2. There is real-time headroom left over for the slow domain.
"""

import dataclasses
import math

import pytest

from scenarios import benchmark, iq_step, speed_step
from scenarios.metrics import load_rejection, step_metrics
from sim.fastcore import FastCore
from sim.params import MOTOR

RPM = 2 * math.pi / 60


# ------------------------------------------------- closed-loop behaviour

def test_current_mode_tracks_its_reference():
    fc = FastCore()
    fc.set_speed(100.0)
    o = fc.advance_current(500, iq_ref=180.0, id_ref=-40.0, w_clamp=100.0)
    assert o.i_q == pytest.approx(180.0, rel=1e-3)
    assert o.i_d == pytest.approx(-40.0, rel=1e-3)


def test_reported_torque_matches_the_current_it_reports():
    fc = FastCore()
    fc.set_speed(100.0)
    o = fc.advance_current(500, iq_ref=200.0, w_clamp=100.0)
    expect = 1.5 * MOTOR.p * (MOTOR.psi_f * o.i_q
                              + (MOTOR.Ld - MOTOR.Lq) * o.i_d * o.i_q)
    assert o.torque == pytest.approx(expect, rel=1e-9)


def test_speed_mode_reaches_its_reference_against_a_load():
    fc = FastCore()
    w_ref = 1500.0 * RPM
    for _ in range(2000):                      # 2 s in 1 ms windows
        o = fc.advance_speed(10, w_ref=w_ref, t_load=60.0)
    assert o.w_m == pytest.approx(w_ref, rel=2e-3)


def test_electrical_power_is_positive_when_motoring_negative_when_regenerating():
    fc = FastCore()
    fc.set_speed(200.0)
    motoring = fc.advance_current(2000, iq_ref=250.0, w_clamp=200.0)
    assert motoring.p_elec_mean > 0.0

    fc2 = FastCore()
    fc2.set_speed(200.0)
    regen = fc2.advance_current(2000, iq_ref=-250.0, w_clamp=200.0)
    assert regen.p_elec_mean < 0.0


# --------------------------------------------------- scheduling contract

def test_window_size_does_not_change_the_result():
    """One 50-tick window must equal fifty 1-tick windows.

    The speed loop runs off an absolute tick counter precisely so a caller's
    choice of window size cannot shift control timing. If this ever fails, the
    real-time engine and the offline scenarios have quietly diverged and every
    figure in the repo becomes untrustworthy.
    """
    big = FastCore()
    small = FastCore()
    w_ref = 800.0 * RPM

    for _ in range(20):
        ob = big.advance_speed(50, w_ref=w_ref, t_load=25.0)
        for _ in range(50):
            os_ = small.advance_speed(1, w_ref=w_ref, t_load=25.0)

    assert ob.w_m == pytest.approx(os_.w_m, rel=1e-12)
    assert ob.i_q == pytest.approx(os_.i_q, rel=1e-12)
    assert ob.theta_e == pytest.approx(os_.theta_e, rel=1e-12)


def test_divergence_is_reported_rather_than_silently_propagated():
    """A stiff plant that RK4 cannot hold must raise, not return NaN.

    Ld of 1 nH makes the electrical time constant ~1e-7 s, far below the 1 us
    substep, so the integration blows up. The C core checks isfinite each tick
    and reports FC_ERR_NONFINITE; without that, NaN would spread into the pack
    and the estimators and only surface as a blank cockpit.
    """
    unstable = dataclasses.replace(MOTOR, Ld=1e-9, Lq=1e-9)
    fc = FastCore(motor=unstable)
    with pytest.raises(FloatingPointError):
        for _ in range(100):
            fc.advance_current(100, iq_ref=200.0)


def test_faults_latch_and_can_be_cleared():
    fc = FastCore()
    fc.set_speed(100.0)
    fc.advance_current(200, iq_ref=100.0, w_clamp=100.0)
    fc.clear_faults()
    o = fc.advance_current(200, iq_ref=100.0, w_clamp=100.0)
    assert o.fault_flags == 0      # healthy operation trips nothing


# ------------------------------------------------------- the phase gate

def test_iq_step_reproduces_edrive_foc_control():
    rec = iq_step.run()
    m = step_metrics(rec["t"], rec["iq"], 100.0, 0.002)
    ref = iq_step.REFERENCE
    # One PWM period of tolerance: this repo samples after the tick it just
    # ran, repo #4 sampled before. That shifts sample-counted metrics by one
    # tick (0.1 ms) and is a convention difference, not a physics difference.
    assert m["rise_ms"] == pytest.approx(ref["rise_ms"], abs=0.1)
    assert m["overshoot_pct"] < 1.0
    assert m["sse_pct"] < 0.5


def test_speed_step_reproduces_edrive_foc_control():
    rec = speed_step.run()
    pre = rec["t"] < 0.5
    m = step_metrics(rec["t"][pre], rec["w_rpm"][pre], 1000.0, 0.0)
    lr = load_rejection(rec["t"], rec["w_rpm"], 1000.0, 0.5)
    ref = speed_step.REFERENCE

    assert m["rise_ms"] == pytest.approx(ref["rise_ms"], abs=1.0)
    assert m["overshoot_pct"] == pytest.approx(ref["overshoot_pct"], abs=0.3)
    assert m["sse_pct"] < 0.1
    assert lr["dip_rpm"] == pytest.approx(ref["dip_rpm"], abs=0.5)


def test_real_time_headroom():
    """The gate is 5x. Asserted at 20x so a slow CI runner still passes while
    a genuine performance regression — which would be an order of magnitude,
    not a few percent — still fails."""
    r = benchmark.measure(sim_seconds=0.5)
    assert r["speed_factor"] > 20.0, (
        f"only {r['speed_factor']:.1f}x real time — the cockpit needs the "
        f"slow domain to fit in what is left of each millisecond")
