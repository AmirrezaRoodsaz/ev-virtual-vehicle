# SESSIONS — running state file

**Read this first. Update it in the last commit of every session.**

Build plan and git protocol: `PROJECT_BRIEF_EV_Virtual_Vehicle.md` §11 and §12
(kept in the parent portfolio workspace, not in this repo).

Protocol per session: branch `sNN-<slug>` → 2–5 commits → `make test` green →
`git merge --no-ff` into `main` with the session log as the merge commit body →
update this file.

| # | Session | Status | Tag |
|---|---------|--------|-----|
| S1 | Bootstrap: venv, Makefile, CI, vendored firmware, smoke test | ✅ done | |
| S2 | `pmsm_plant.c` — dq plant ported to C | ✅ done | |
| S3 | `fastcore.c` — `advance_n()`, reproduce repo #4 figures, benchmark | ✅ done | `v0.1-fastcore` |
| S4 | `sim/vehicle.py` — longitudinal dynamics | ⬜ next | |
| S5 | `sim/pack.py` — 96-cell vectorized 2-RC ECM | ⬜ | |
| S6 | `sim/thermal.py` + `sim/aging.py` | ⬜ | |
| S7 | `sim/engine.py` — three-domain scheduler, WLTP run | ⬜ | `v0.2-plant` |
| S8 | `bms/sensors.py` + `sim/faults.py` — the sensor boundary | ⬜ | |
| S9 | `bms/ekf.py` — EKF SOC (headline figure) | ⬜ | |
| S10 | `bms/soh_physics.py` + `bms/limits.py` — derating closes the loop | ⬜ | `v0.3-bms` |
| S11 | `can/` — extended DBC + virtual bus + decoder client | ⬜ | `v0.4-can` |
| S12 | `server/app.py` — FastAPI + WebSocket + pacing | ⬜ | |
| S13 | `server/static/` — cockpit UI (**demo-ready stop point**) | ⬜ | `v0.5-cockpit` |
| S14 | `ml/features.py` — Severson data prep | ⬜ | |
| S15 | `ml/train.py` — training + honest eval | ⬜ | |
| S16 | `bms/soh_ml.py` + `reports/domain_gap.md` | ⬜ | `v0.6-ml` |
| S17 | README, figures, release | ⬜ | `v1.0` |

---

## S1 — Bootstrap (done)

**Shipped:** Python 3.11 venv project skeleton; the FOC control stack vendored
unchanged from `edrive-foc-control@086e803`; a `Makefile` that compiles the C
into a host shared library; a ctypes smoke test that loads that library and
exercises the vendored transforms; GitHub Actions CI running `make` + `pytest`
on every push.

**Decided:** vendored rather than submoduled, so the repo clones and runs
standalone — the source commit is pinned in `firmware/VENDORED.md`. Vendored C
files are treated as **read-only**: bug fixes go upstream to
`edrive-foc-control` and get re-vendored, so the two repos never diverge
silently.

**Deferred:** nothing.

**S2 starts from:** `firmware/pmsm_plant.c`. Port the dq PMSM model from
`edrive-foc-control/sim/pmsm.py` into C. Motor parameters and their citation
live in `edrive-foc-control/sim/params.py` (35 kW / 200 Nm 8-pole IPMSM,
Somefun & Longe 2026). Validate the C plant against analytic steady-state
operating points through ctypes before wiring it to the controller in S3.

---

## S2 — dq PMSM plant in C (done)

**Shipped:** `firmware/pmsm_plant.c/.h` — electrical dq dynamics, torque with
the reluctance term, mechanical ODE, RK4 fixed-step integration, the ideal
inverter (duties → average dq voltage) and the phase-current sensor view.
`sim/params.py` with the cited motor and derived gains. Plant bindings in
`sim/clib.py`. 11 new tests, 16 total, all green.

**Decided:** the plant is `double` while the vendored controller stays `float`.
The controller is single precision because a real motor MCU is; the plant is
the reference physics that controller is judged against, so it must not
contribute the error it is meant to measure. It nonetheless lives in
`firmware/` because compiling into the same shared library is what buys the
real-time headroom in S3 — the header comment marks the boundary.

`theta_m` is wrapped to [0, 2π). Integer pole pairs mean this changes no
observable, but it keeps absolute angle precision bounded over a long drive.

**Found:** the rotating steady-state test initially failed at 0.18% error. Not
a tolerance problem — 0.2 s is only ~6.8 electrical time constants and the
residual exponential was still larger than the tolerance. Now integrates 1.0 s
(~34 τ). Worth remembering: this machine's electrical time constant is ~29.5 ms,
which is slow enough to matter in later settling tests.

**Deferred:** nothing.

**S3 starts from:** `firmware/fastcore.c`. Write `advance_n(n_steps, ...)` that
runs `foc_step` (and `spd_step` on its divider) against `pmsm_plant` entirely
inside C, so Python crosses the ctypes boundary once per millisecond rather
than once per 100 µs. Then reproduce `edrive-foc-control`'s torque-step and
speed-step figures through the new core, and benchmark simulated-seconds per
wall-second. Gate for the phase tag: figures match **and** ≥5× real-time
headroom. The vendored `Spd`/`Fd` structs are not yet mirrored in `sim/clib.py`
— S3 adds them.

---

## S3 — the fast domain closed inside C (done) — Phase 1 complete, `v0.1-fastcore`

**Shipped:** `firmware/fastcore.c/.h` running whole PWM periods without
returning to Python; `sim/fastcore.py` as its window-based Python face;
`scenarios/iq_step.py`, `speed_step.py`, `benchmark.py`; both figures
committed. 10 new tests, 26 total, green.

**Gate met, both halves:**

| | this repo | edrive-foc-control |
|---|---|---|
| i_q step rise | 0.600 ms | 0.600 ms |
| i_q step overshoot | 0.0% | 0.2% |
| speed step rise | 66.0 ms | 65.8 ms |
| speed step overshoot | 0.6% | 0.6% |
| 50 N m load dip | 8.3 rpm | 8.3 rpm |
| real-time factor | **200x** (gate: 5x) | — |

5 µs per 1 ms window, leaving 995 µs of every millisecond for the slow domain.
That is a very comfortable budget for S4–S13 — the vehicle, 96-cell pack,
thermal, aging, EKF, CAN and render all have to fit inside it, and now
plainly will.

**Decided:** the speed divider counts *absolute* ticks, not ticks within a
window, so a caller's choice of window size cannot shift control timing. The
test that one 50-tick window equals fifty 1-tick windows to twelve digits
guards this. If it ever fails, the real-time engine and the offline scenarios
have diverged and every figure in the repo is untrustworthy.

`fc_t` is opaque to Python (allocate `fc_sizeof()` bytes) so adding controller
state in C cannot silently corrupt the binding.

**Known convention difference:** this repo samples after the tick just run;
repo #4 sampled before. That shifts sample-counted metrics (settling time,
steady-state error at the final sample) by one PWM period. It is not a physics
difference and the tests allow exactly one tick for it.

**Deferred:** sensor-fault injection hooks in the fast domain. S8 owns the
`Sensors` boundary and will add current-sensor and encoder corruption inside
`fc_advance` then, where it belongs, rather than being guessed at now.

**S4 starts from:** `sim/vehicle.py` — longitudinal dynamics (mass, rolling
resistance, aero drag, road grade, gear ratio, regen blend) reflected to the
motor shaft as the `t_load` that `fc_advance` already accepts. Parameters are
an ASSUMED generic 1500 kg compact EV and must say so where they are defined,
per the rule in `sim/params.py`. Validate against an analytic coastdown.
