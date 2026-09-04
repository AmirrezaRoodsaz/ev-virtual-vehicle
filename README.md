# ev-virtual-vehicle

**A drivable, real-time digital twin of an electric powertrain.**

Hold a key and a simulated EV accelerates — with production-shaped embedded C
control code in the loop at 10 kHz, a 96-cell electro-thermal battery pack, and
a BMS whose estimators only ever see noisy sensors. Flip a fault switch
mid-drive and watch the consequences propagate.

> 🚧 **Under construction.** Built session by session; see `SESSIONS.md` for
> current state. This README is a stub until S17.

## Status

Session 1 of 17 complete: project skeleton, vendored FOC firmware, build, CI.

## Related repositories

This is the integration layer for a four-repo portfolio:

- [`edrive-foc-control`](https://github.com/AmirrezaRoodsaz/edrive-foc-control) — the FOC firmware vendored here
- [`ev-can-bms-telemetry`](https://github.com/AmirrezaRoodsaz/ev-can-bms-telemetry) — CAN/DBC foundation
- [`battery-rul-prediction`](https://github.com/AmirrezaRoodsaz/battery-rul-prediction) — battery ML approach reused for the SOH head
- [`battery-docs-rag-agent`](https://github.com/AmirrezaRoodsaz/battery-docs-rag-agent) — GenAI sibling
