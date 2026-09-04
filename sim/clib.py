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
    return lib


def out2():
    """Two fresh c_float outputs plus their pointers, for the transform calls."""
    a, b = ctypes.c_float(), ctypes.c_float()
    return a, b, ctypes.byref(a), ctypes.byref(b)
