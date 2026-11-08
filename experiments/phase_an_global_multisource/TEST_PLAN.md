# Phase AN test plan

Parent: pushed AM d84d8a387f56e9599f45638505a1856914f09415. Keep local15D immutable. Consume original AL284-frame native window, AM284 support records and five actual multistart batches. Extract both raw GNSS sources from the same bag/window read-only using installed ROS2 type support; check enabled fields,fix,receipt/Header timing and coordinates without exporting coordinates.

Implement global health,dimension-specific coordinate anchors and discrete mode handling without inventing NDT R. Explicit prerequisites for position anchor: datum/reference/time/fix qualification. Pruning additionally needs independently validated joint deterministic mismatch bounds and independent source evidence. No empirical uncertainty or result-dependent weighting. No qualified real input is invented if prerequisites fail.

10 new automatic tests cover all lifecycle states,position-only/yaw protection,old-dimensional expiry,duplicate/stale/clock rollback,contract checks,atomic bad-bound rejection,local isolation,WGS84/NED math and same-time mode transport. Separate executable synthetic recovery fixture. Real A/B/C comparison must expose incomplete historical runtime/map/acceptance metadata. ROS2 isolated AN transport should reproduce offline state/reason counts and publish no unconfirmed global anchor. No long replay or parameter search.
