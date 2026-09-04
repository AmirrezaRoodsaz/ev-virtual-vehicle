"""Python face of the C fast domain.

Owns one opaque `fc_t` and hands it commands in *windows*: a caller says
"advance one millisecond" and receives the state at the end plus the means
over it. The real-time engine uses 1 ms windows, which is what keeps the
boundary crossings at 1,000 per simulated second instead of 10,000.

Offline scenarios may pass `n_ticks=1` when a figure needs per-PWM-period
resolution. That is legitimate — they are not racing a wall clock — but it
pays exactly the cost the C core exists to remove, so the real-time path must
never do it.
"""

import ctypes

from sim import clib
from sim.params import CTRL, MOTOR, ControlParams, MotorParams

N_SUB = 10  # plant RK4 substeps per PWM period -> 1 us step at 10 kHz


class FastCore:
    """Controller + plant closed inside C at the PWM rate."""

    def __init__(self, motor: MotorParams = MOTOR, ctrl: ControlParams = CTRL,
                 n_sub: int = N_SUB, lib=None):
        self.lib = clib.load() if lib is None else lib
        self.motor, self.ctrl, self.n_sub = motor, ctrl, n_sub

        # The C struct is opaque: allocate what it reports it needs, so adding
        # controller state in C can never silently corrupt this binding.
        self._buf = ctypes.create_string_buffer(self.lib.fc_sizeof())
        self._fc = ctypes.cast(self._buf, ctypes.c_void_p)

        cfg = clib.FcConfig(v_dc=motor.v_dc, dt=ctrl.ts, n_sub=n_sub,
                            speed_div=ctrl.speed_div)
        cg = ctrl.current_loop_gains(motor)
        foc_par = clib.FocParams(
            rs=motor.Rs, ld=motor.Ld, lq=motor.Lq, psi_f=motor.psi_f,
            kp_d=cg["kp_d"], ki_d=cg["ki_d"],
            kp_q=cg["kp_q"], ki_q=cg["ki_q"], ts=ctrl.ts)
        sg = ctrl.speed_loop_gains(motor)
        spd_par = clib.SpdParams(kp=sg["kp"], ki=sg["ki"], ts=sg["ts"],
                                 t_filt=sg["t_filt"], iq_max=sg["iq_max"])
        # Thresholds carried over from edrive-foc-control's phase 4.
        fd_par = clib.FdParams(ts=ctrl.ts, sum_thresh=5.0,
                               mag_thresh=motor.i_max * 1.15,
                               ang_thresh=0.05, debounce=3)

        self.lib.fc_init(self._fc, ctypes.byref(cfg),
                         ctypes.byref(clib.motor_params(motor)),
                         ctypes.byref(foc_par), ctypes.byref(spd_par),
                         ctypes.byref(fd_par))
        self.out = clib.FcOut()

    # ---- commands ----

    def set_speed(self, w_m: float) -> None:
        """Place the rotor at a mechanical speed without integrating up to it."""
        self.lib.fc_set_speed(self._fc, w_m)

    def clear_faults(self) -> None:
        self.lib.fc_clear_faults(self._fc)

    def _advance(self, cmd: "clib.FcCmd", n_ticks: int) -> "clib.FcOut":
        status = self.lib.fc_advance(self._fc, ctypes.byref(cmd), n_ticks,
                                     ctypes.byref(self.out))
        if status == clib.FC_ERR_NONFINITE:
            raise FloatingPointError(
                "plant state diverged inside the C fast domain — the "
                "simulation cannot continue from here")
        return self.out

    def advance_current(self, n_ticks: int, iq_ref: float, id_ref: float = 0.0,
                        t_load: float = 0.0, w_clamp: float | None = None):
        """Dyno mode: command currents directly.

        `w_clamp` holds the shaft at a fixed speed, isolating the current
        loops from the mechanics — how the current-loop figures are made.
        """
        cmd = clib.FcCmd(mode=clib.FC_MODE_CURRENT, id_ref=id_ref,
                         iq_ref=iq_ref, w_ref=0.0, t_load=t_load,
                         clamp_speed=int(w_clamp is not None),
                         w_clamp=0.0 if w_clamp is None else w_clamp)
        return self._advance(cmd, n_ticks)

    def advance_speed(self, n_ticks: int, w_ref: float, t_load: float = 0.0):
        """Cascade mode: the outer speed loop generates iq_ref."""
        cmd = clib.FcCmd(mode=clib.FC_MODE_SPEED, id_ref=0.0, iq_ref=0.0,
                         w_ref=w_ref, t_load=t_load, clamp_speed=0,
                         w_clamp=0.0)
        return self._advance(cmd, n_ticks)

    # ---- convenience ----

    @property
    def dt_tick(self) -> float:
        return self.ctrl.ts
