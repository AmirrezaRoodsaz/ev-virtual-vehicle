"""Step-response metrics.

Definitions are copied from `edrive-foc-control/scenarios/` deliberately and
must not be "improved" here: S3's phase gate compares this repo's numbers
against that repo's published ones, and a comparison is only meaningful if
both sides measure the same way.
"""

import numpy as np


def step_metrics(t, y, target, t_on):
    """Rise (10-90%), overshoot, 2% settling time, steady-state error."""
    on = t >= t_on
    t_step, y_step = t[on], y[on]
    rise = (t_step[np.argmax(y_step >= 0.9 * target)]
            - t_step[np.argmax(y_step >= 0.1 * target)])
    overshoot = max(0.0, (y.max() - target) / target * 100.0)
    sse = abs(y[-1] - target) / target * 100.0
    band = np.abs(y - target) <= 0.02 * target
    settle = t[~band][-1] - t_on if (~band).any() else 0.0
    return {"rise_ms": rise * 1e3, "overshoot_pct": overshoot,
            "settle_ms": settle * 1e3, "sse_pct": sse}


def load_rejection(t, w_rpm, w_ref_rpm, t_load_on):
    """Speed dip under a load-torque step, and time to recover into 1%."""
    after = t >= t_load_on
    dip = w_ref_rpm - w_rpm[after].min()
    off = after & (np.abs(w_rpm - w_ref_rpm) > 0.01 * w_ref_rpm)
    recovery = (t[off][-1] - t_load_on) if off.any() else 0.0
    return {"dip_rpm": dip, "recovery_ms": recovery * 1e3}
