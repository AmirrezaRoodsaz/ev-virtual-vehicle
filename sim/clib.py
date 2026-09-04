"""ctypes loader for the C fast domain.

One place that knows where the shared library lives and how its C structs are
laid out, so every later module (plant validation in S2, the co-sim scheduler
in S3, the real-time engine in S7) shares a single binding.

Struct definitions must mirror `firmware/*.h` field-for-field and in order —
ctypes cannot check this, so the smoke test exercises a round-trip through the
C to catch a mismatch immediately.
"""

import ctypes
import pathlib
import platform

_LIBNAME = "libevtwin.dylib" if platform.system() == "Darwin" else "libevtwin.so"
_LIBPATH = pathlib.Path(__file__).resolve().parent.parent / "build" / _LIBNAME


# ---- struct mirrors of firmware/foc.h ----

class FocPi(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in ("kp", "ki", "ts", "integ")]


class FocParams(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ("rs", "ld", "lq", "psi_f",
                 "kp_d", "ki_d", "kp_q", "ki_q", "ts")]


class Foc(ctypes.Structure):
    _fields_ = [("par", FocParams), ("pi_d", FocPi), ("pi_q", FocPi)]


class FocIn(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ("ia", "ib", "ic", "theta_e", "w_e",
                 "id_ref", "iq_ref", "v_dc")]


class FocOut(ctypes.Structure):
    _fields_ = [("duty_a", ctypes.c_float), ("duty_b", ctypes.c_float),
                ("duty_c", ctypes.c_float),
                ("id", ctypes.c_float), ("iq", ctypes.c_float),
                ("vd", ctypes.c_float), ("vq", ctypes.c_float),
                ("sat", ctypes.c_int)]


_F = ctypes.c_float
_PF = ctypes.POINTER(ctypes.c_float)
_D = ctypes.c_double
_PD = ctypes.POINTER(ctypes.c_double)

# ---- struct mirrors of firmware/pmsm_plant.h (owned here, not vendored) ----
# Note the type change: the plant is double, the controller float. See the
# header comment in pmsm_plant.h for why.

class PmsmParams(ctypes.Structure):
    _fields_ = [("rs", ctypes.c_double), ("ld", ctypes.c_double),
                ("lq", ctypes.c_double), ("psi_f", ctypes.c_double),
                ("j", ctypes.c_double), ("b", ctypes.c_double),
                ("p", ctypes.c_int)]


class PmsmState(ctypes.Structure):
    _fields_ = [("i_d", ctypes.c_double), ("i_q", ctypes.c_double),
                ("w_m", ctypes.c_double), ("theta_m", ctypes.c_double)]


def motor_params(m=None):
    """Build the C plant parameter struct from sim.params.MotorParams."""
    from sim.params import MOTOR
    m = MOTOR if m is None else m
    return PmsmParams(rs=m.Rs, ld=m.Ld, lq=m.Lq, psi_f=m.psi_f,
                      j=m.J, b=m.B, p=m.p)



def load():
    """Load the shared library with argtypes bound. Raises if `make` hasn't run."""
    if not _LIBPATH.exists():
        raise FileNotFoundError(
            f"{_LIBPATH} not found — run `make` to build the C fast domain first."
        )
    lib = ctypes.CDLL(str(_LIBPATH))

    lib.foc_clarke.argtypes = [_F, _F, _F, _PF, _PF]
    lib.foc_clarke.restype = None
    lib.foc_park.argtypes = [_F, _F, _F, _F, _PF, _PF]
    lib.foc_park.restype = None
    lib.foc_inv_park.argtypes = [_F, _F, _F, _F, _PF, _PF]
    lib.foc_inv_park.restype = None
    lib.foc_svpwm.argtypes = [_F, _F, _F, _PF, _PF, _PF]
    lib.foc_svpwm.restype = None
    lib.foc_pi_step.argtypes = [ctypes.POINTER(FocPi), _F, _F, _F]
    lib.foc_pi_step.restype = _F
    lib.foc_init.argtypes = [ctypes.POINTER(Foc), ctypes.POINTER(FocParams)]
    lib.foc_init.restype = None
    lib.foc_step.argtypes = [ctypes.POINTER(Foc), ctypes.POINTER(FocIn),
                             ctypes.POINTER(FocOut)]
    lib.foc_step.restype = None

    lib.pmsm_init.argtypes = [ctypes.POINTER(PmsmState)]
    lib.pmsm_init.restype = None
    lib.pmsm_torque.argtypes = [ctypes.POINTER(PmsmParams), _D, _D]
    lib.pmsm_torque.restype = _D
    lib.pmsm_theta_e.argtypes = [ctypes.POINTER(PmsmParams),
                                 ctypes.POINTER(PmsmState)]
    lib.pmsm_theta_e.restype = _D
    lib.pmsm_w_e.argtypes = [ctypes.POINTER(PmsmParams),
                             ctypes.POINTER(PmsmState)]
    lib.pmsm_w_e.restype = _D
    lib.pmsm_step.argtypes = [ctypes.POINTER(PmsmParams),
                              ctypes.POINTER(PmsmState), _D, _D, _D, _D]
    lib.pmsm_step.restype = None
    lib.pmsm_duties_to_vdq.argtypes = [ctypes.POINTER(ctypes.c_double * 3),
                                       _D, _D, _PD, _PD]
    lib.pmsm_duties_to_vdq.restype = None
    lib.pmsm_dq_to_phase.argtypes = [_D, _D, _D, _PD, _PD, _PD]
    lib.pmsm_dq_to_phase.restype = None
    return lib


def out2():
    """Two fresh c_float outputs plus their pointers, for the transform calls."""
    a, b = ctypes.c_float(), ctypes.c_float()
    return a, b, ctypes.byref(a), ctypes.byref(b)


def dout(n):
    """n fresh c_double outputs plus their pointers, for the plant calls."""
    vals = [ctypes.c_double() for _ in range(n)]
    return vals, [ctypes.byref(v) for v in vals]
