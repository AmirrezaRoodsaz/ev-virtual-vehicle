"""Physical parameters for the twin, and their provenance.

**Cited vs. assumed.** Only the motor is taken from a published source. The
vehicle, pack, thermal and aging parameters added in later sessions are
*assumed* values for a generic compact EV, and every one of them says so in
its own docstring. The README repeats the distinction — a reader must never
have to guess which numbers are defensible and which are plausible.

Motor: 35 kW / 200 N m 8-pole interior PMSM for an EV powertrain.
Source: Somefun & Longe, "Thermal and dynamic modelling of permanent-magnet
synchronous motors (PMSM) driven electric vehicle powertrain with dual-loop
control strategy", Frontiers in Energy Research, 2026,
doi:10.3389/fenrg.2026.1832111 (Table 2).

Controller gains are DERIVED from the motor parameters, not hand-tuned. The
derivations are carried over unchanged from `edrive-foc-control` so both
repositories tune the identical firmware identically.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MotorParams:
    """35 kW / 200 N m 8-pole IPMSM. Cited except where noted."""

    # Electrical
    Rs: float = 0.010087        # stator phase resistance [ohm]
    Ld: float = 243.68e-6       # d-axis inductance [H]
    Lq: float = 297.58e-6       # q-axis inductance [H]
    psi_f: float = 0.0440       # PM flux linkage [Wb]
    p: int = 4                  # pole pairs (8 poles)

    # Ratings
    i_max: float = 425.0 * math.sqrt(2.0)   # max phase current, peak [A]
    t_max: float = 200.0        # max torque [N m]
    v_dc: float = 325.0         # nominal DC-link voltage [V]

    # Mechanical
    J: float = 0.1243           # rotor inertia [kg m^2]
    B: float = 1e-3             # viscous friction [N m s/rad] — ASSUMED,
                                # not given in the source


@dataclass(frozen=True)
class ControlParams:
    """Everything the firmware needs, derived from MotorParams."""

    f_pwm: float = 10_000.0     # PWM / current-ISR rate [Hz] (cited: 10 kHz)
    speed_div: int = 10         # speed loop every N current ticks -> 1 kHz

    @property
    def ts(self) -> float:
        return 1.0 / self.f_pwm

    def current_loop_gains(self, m: MotorParams, f_bw: float = 500.0) -> dict:
        """Modulus-optimum-style tuning of the PI current controllers.

        Plant per axis after decoupling: G(s) = 1/(L s + Rs). Placing the PI
        zero on the plant pole (Ki/Kp = Rs/L) cancels it, leaving Kp/(L s) —
        a pure integrator — so the closed loop is first order with bandwidth
        w_c = Kp/L. Hence Kp = w_c L and Ki = w_c Rs.

        f_bw = 500 Hz = f_pwm/20 sits safely below the Nyquist and delay
        limits of a 10 kHz ISR; standard practice for digital current loops.
        """
        wc = 2.0 * math.pi * f_bw
        return {
            "kp_d": wc * m.Ld, "ki_d": wc * m.Rs,
            "kp_q": wc * m.Lq, "ki_q": wc * m.Rs,
            "w_bw": wc,
        }

    def speed_loop_gains(self, m: MotorParams, a: float = 2.0) -> dict:
        """Symmetric-optimum tuning of the outer speed PI.

        The speed loop sees the closed current loop (first-order lag
        tau_i = 1/w_bw), the torque constant Kt = 1.5 p psi_f, and the
        mechanical integrator 1/(J s). An integrating plant rules out
        pole-zero cancellation — it would destroy load-disturbance
        rejection — so the symmetric optimum instead centres the crossover
        geometrically between the PI corner and the parasitic lag:

            tau_eq = tau_i + Ts_speed
            Ti     = a^2 tau_eq        (a = 2 -> phase margin ~37 deg)
            Kp     = J / (a tau_eq Kt)

        The PI zero would make reference steps overshoot ~43%; a
        first-order prefilter with T = Ti cancels it and drops that to
        under 1% without touching disturbance stiffness.
        """
        tau_i = 1.0 / self.current_loop_gains(m)["w_bw"]
        ts_spd = self.ts * self.speed_div
        tau_eq = tau_i + ts_spd
        kt = 1.5 * m.p * m.psi_f
        kp = m.J / (a * tau_eq * kt)
        ti = a * a * tau_eq
        return {
            "kp": kp, "ki": kp / ti, "ts": ts_spd,
            "t_filt": ti, "iq_max": m.i_max, "kt": kt, "tau_eq": tau_eq,
        }


MOTOR = MotorParams()
CTRL = ControlParams()
