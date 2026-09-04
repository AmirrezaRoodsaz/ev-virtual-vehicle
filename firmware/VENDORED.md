# Vendored firmware

`foc.c/.h`, `speed_ctrl.c/.h` and `fault_detect.c/.h` are copied **unchanged**
from a sibling portfolio repository.

| | |
|---|---|
| Source | https://github.com/AmirrezaRoodsaz/edrive-foc-control |
| Commit | `086e803f3d311b54d8bdd9db10eefae13f8695f9` |
| Path | `firmware/` |
| Vendored on | 2026-08-28 |

## Why vendored and not a submodule

This repo must clone and run standalone — one `git clone`, one `make`, one
`pytest`. A submodule buys version tracking at the cost of every reader
hitting an empty directory on first clone. The commit hash above provides the
traceability a submodule would have given.

## Rule: these files are read-only here

Do not edit the vendored sources in this repository. A bug found here is a bug
in `edrive-foc-control` — fix it upstream, then re-vendor and bump the commit
hash above. This keeps the two repositories from silently diverging, and keeps
the claim "the same control code drives both projects" literally true.

New C written *for this project* (`pmsm_plant.c`, `fastcore.c`) is not vendored
and is owned here.

## What each file provides

- **`foc.c`** — Clarke/Park/inverse-Park transforms, PI current controllers
  with conditional anti-windup, SVPWM duty computation, and the complete
  `foc_step()` current-loop update designed to be called from a PWM-rate ISR.
- **`speed_ctrl.c`** — outer symmetric-optimum speed PI with reference
  prefilter, commanding `iq_ref` under a current limit.
- **`fault_detect.c`** — debounced plausibility monitoring (phase-current sum,
  current magnitude, encoder consistency) with latched fault flags.
