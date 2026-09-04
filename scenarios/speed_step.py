"""S3 gate, part 2: reproduce edrive-foc-control's cascade speed step.

0 -> 1000 rpm reference step, then a 50 N m load-torque step at t = 0.5 s.
Exercises the outer speed loop, its reference prefilter, the current limit,
and load-disturbance rejection — all now scheduled inside C.

Run:  .venv/bin/python -m scenarios.speed_step
"""

import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scenarios.metrics import load_rejection, step_metrics
from sim.fastcore import FastCore
from sim.params import CTRL

FIGDIR = pathlib.Path(__file__).resolve().parent.parent / "reports/figures"
RPM = 2 * np.pi / 60

# Published in edrive-foc-control's README, regenerated 2026-08-28.
REFERENCE = {"rise_ms": 65.8, "overshoot_pct": 0.6, "sse_pct": 0.0,
             "dip_rpm": 8.3}

TICKS_PER_SAMPLE = 10   # 1 ms — the real-time engine's window size


def run(w_ref_rpm=1000.0, t_end=1.0, t_load_on=0.5, t_load_nm=50.0):
    fc = FastCore()
    n = int(round(t_end / (CTRL.ts * TICKS_PER_SAMPLE)))
    dt = CTRL.ts * TICKS_PER_SAMPLE
    rec = {k: np.zeros(n) for k in ("t", "w_rpm", "iq", "iq_ref", "id",
                                    "te", "p_kw")}
    for k in range(n):
        t = (k + 1) * dt
        load = t_load_nm if t >= t_load_on else 0.0
        o = fc.advance_speed(TICKS_PER_SAMPLE, w_ref=w_ref_rpm * RPM,
                             t_load=load)
        rec["t"][k] = t
        rec["w_rpm"][k] = o.w_m / RPM
        rec["id"][k], rec["iq"][k] = o.i_d, o.i_q
        rec["iq_ref"][k] = o.iq_ref
        rec["te"][k] = o.torque
        rec["p_kw"][k] = o.p_elec_mean / 1e3
    return rec


def main():
    w_ref, t_on, t_nm = 1000.0, 0.5, 50.0
    rec = run(w_ref, t_load_on=t_on, t_load_nm=t_nm)
    pre = rec["t"] < t_on
    m = step_metrics(rec["t"][pre], rec["w_rpm"][pre], w_ref, 0.0)
    lr = load_rejection(rec["t"], rec["w_rpm"], w_ref, t_on)

    print(f"speed step {w_ref:.0f} rpm | rise {m['rise_ms']:.1f} ms "
          f"| overshoot {m['overshoot_pct']:.1f}% "
          f"| ss err {m['sse_pct']:.3f}% "
          f"| {t_nm:.0f} Nm dip {lr['dip_rpm']:.1f} rpm")
    print("reference (edrive-foc-control): "
          + " | ".join(f"{k} {v}" for k, v in REFERENCE.items()))

    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    ax[0].axhline(w_ref, color="k", ls="--", lw=1, label="reference")
    ax[0].plot(rec["t"], rec["w_rpm"], label=r"$\omega_m$")
    ax[0].set_ylabel("speed [rpm]")
    ax[0].set_title(f"Cascade speed step to {w_ref:.0f} rpm with a "
                    f"{t_nm:.0f} N m load step at {t_on:.1f} s\n"
                    f"rise {m['rise_ms']:.1f} ms, "
                    f"overshoot {m['overshoot_pct']:.1f}%, "
                    f"dip {lr['dip_rpm']:.1f} rpm "
                    f"(reference {REFERENCE['rise_ms']:.1f} ms, "
                    f"{REFERENCE['overshoot_pct']:.1f}%, "
                    f"{REFERENCE['dip_rpm']:.1f} rpm)")
    ax[0].legend()
    ax[1].plot(rec["t"], rec["iq_ref"], "k--", lw=1, label=r"$i_q^{ref}$")
    ax[1].plot(rec["t"], rec["iq"], label=r"$i_q$")
    ax[1].plot(rec["t"], rec["te"], label=r"$T_e$ [N m]")
    ax[1].set_ylabel("current [A] / torque [N m]")
    ax[1].legend()
    ax[2].plot(rec["t"], rec["p_kw"])
    ax[2].set_ylabel("electrical power [kW]")
    ax[2].set_xlabel("time [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    FIGDIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGDIR / "speed_step.png", dpi=150)
    print(f"figure -> {FIGDIR / 'speed_step.png'}")


if __name__ == "__main__":
    main()
