"""S3 gate, part 1: reproduce edrive-foc-control's q-axis current step.

100 A step at 1000 rpm with the shaft clamped, so the mechanics are removed
and this measures the current loops alone. The controller and its gains are
identical to repo #4; what changed is that the loop now closes inside C
against a C plant. The metrics must match, or the port broke something.

Run:  .venv/bin/python -m scenarios.iq_step
"""

import ctypes
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scenarios.metrics import step_metrics
from sim import clib
from sim.fastcore import FastCore
from sim.params import CTRL

FIGDIR = pathlib.Path(__file__).resolve().parent.parent / "reports/figures"
RPM = 2 * np.pi / 60

# Published in edrive-foc-control's README, regenerated 2026-08-28.
REFERENCE = {"rise_ms": 0.600, "overshoot_pct": 0.2,
             "settle_ms": 0.900, "sse_pct": 0.20}


def run(iq_step=100.0, rpm=1000.0, t_end=0.01, t_on=0.002):
    """Per-PWM-period resolution: n_ticks=1 windows. Offline only."""
    fc = FastCore()
    w_m = rpm * RPM
    fc.set_speed(w_m)

    n = int(round(t_end / CTRL.ts))
    n_on = int(round(t_on / CTRL.ts))
    rec = {k: np.zeros(n) for k in
           ("t", "iq_ref", "id", "iq", "vd", "vq", "ia", "ib", "ic")}

    lib = fc.lib
    for k in range(n):
        iq_ref = iq_step if k >= n_on else 0.0
        o = fc.advance_current(1, iq_ref=iq_ref, w_clamp=w_m)
        rec["t"][k] = (k + 1) * CTRL.ts
        rec["iq_ref"][k] = iq_ref
        rec["id"][k], rec["iq"][k] = o.i_d, o.i_q
        rec["vd"][k], rec["vq"][k] = o.vd, o.vq
        vals, ptrs = clib.dout(3)
        lib.pmsm_dq_to_phase(o.i_d, o.i_q, o.theta_e, *ptrs)
        rec["ia"][k], rec["ib"][k], rec["ic"][k] = (v.value for v in vals)
    return rec


def main():
    step, rpm, t_on = 100.0, 1000.0, 0.002
    rec = run(step, rpm, t_on=t_on)
    m = step_metrics(rec["t"], rec["iq"], step, t_on)

    print(f"iq step {step:.0f} A @ {rpm:.0f} rpm | rise {m['rise_ms']:.3f} ms "
          f"| overshoot {m['overshoot_pct']:.1f}% "
          f"| settle(2%) {m['settle_ms']:.3f} ms "
          f"| ss error {m['sse_pct']:.2f}%")
    print("reference (edrive-foc-control): "
          + " | ".join(f"{k} {v}" for k, v in REFERENCE.items()))

    t_ms = rec["t"] * 1e3
    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(t_ms, rec["iq_ref"], "k--", lw=1, label=r"$i_q^{ref}$")
    ax[0].plot(t_ms, rec["iq"], label=r"$i_q$")
    ax[0].plot(t_ms, rec["id"], label=r"$i_d$")
    ax[0].set_ylabel("current [A]")
    ax[0].set_title(f"q-axis current step, {step:.0f} A at {rpm:.0f} rpm, "
                    f"closed inside the C fast domain\n"
                    f"rise {m['rise_ms']:.2f} ms, "
                    f"overshoot {m['overshoot_pct']:.1f}% "
                    f"(reference {REFERENCE['rise_ms']:.2f} ms, "
                    f"{REFERENCE['overshoot_pct']:.1f}%)")
    ax[0].legend()
    ax[1].plot(t_ms, rec["ia"], label=r"$i_a$")
    ax[1].plot(t_ms, rec["ib"], label=r"$i_b$")
    ax[1].plot(t_ms, rec["ic"], label=r"$i_c$")
    ax[1].set_ylabel("phase current [A]")
    ax[1].legend(ncol=3)
    ax[2].plot(t_ms, rec["vd"], label=r"$v_d$")
    ax[2].plot(t_ms, rec["vq"], label=r"$v_q$")
    ax[2].set_ylabel("voltage [V]")
    ax[2].set_xlabel("time [ms]")
    ax[2].legend()
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    FIGDIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGDIR / "iq_step.png", dpi=150)
    print(f"figure -> {FIGDIR / 'iq_step.png'}")


if __name__ == "__main__":
    main()
