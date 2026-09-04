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
| S2 | `pmsm_plant.c` — dq plant ported to C | ⬜ next | |
| S3 | `fastcore.c` — `advance_n()`, reproduce repo #4 figures, benchmark | ⬜ | `v0.1-fastcore` |
| S4 | `sim/vehicle.py` — longitudinal dynamics | ⬜ | |
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
