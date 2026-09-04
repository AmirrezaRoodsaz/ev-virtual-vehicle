"""S3 gate, part 3: how much faster than real time is the fast domain?

The cockpit must sustain 10 kHz of closed-loop control plus 100 kHz of plant
integration while Python still runs the vehicle, pack, thermal, aging, BMS and
CAN layers on top, and renders. This measures what the C core leaves for them.

Reported as a speed factor: simulated seconds per wall-clock second, driving
the same 1 ms windows the real-time engine will use.

Run:  .venv/bin/python -m scenarios.benchmark
"""

import time

from sim.fastcore import FastCore
from sim.params import CTRL

RPM = 2 * 3.141592653589793 / 60
WINDOW_TICKS = 10        # 1 ms, the real-time engine's window
SIM_SECONDS = 2.0


def measure(sim_seconds=SIM_SECONDS, window=WINDOW_TICKS):
    fc = FastCore()
    n_windows = int(round(sim_seconds / (CTRL.ts * window)))

    t0 = time.perf_counter()
    for _ in range(n_windows):
        fc.advance_speed(window, w_ref=3000.0 * RPM, t_load=80.0)
    wall = time.perf_counter() - t0

    return {
        "sim_s": n_windows * CTRL.ts * window,
        "wall_s": wall,
        "speed_factor": (n_windows * CTRL.ts * window) / wall,
        "us_per_window": wall / n_windows * 1e6,
        "pwm_ticks": n_windows * window,
        "plant_steps": n_windows * window * fc.n_sub,
    }


def main():
    r = measure()
    print(f"simulated {r['sim_s']:.2f} s in {r['wall_s']:.3f} s wall  "
          f"->  {r['speed_factor']:.1f}x real time")
    print(f"  {r['us_per_window']:.1f} us per 1 ms window  "
          f"({r['pwm_ticks']:,} PWM ticks, {r['plant_steps']:,} RK4 steps)")
    budget = 1000.0 - r["us_per_window"]
    print(f"  leaves {budget:.0f} us of every millisecond for the slow domain")


if __name__ == "__main__":
    main()
