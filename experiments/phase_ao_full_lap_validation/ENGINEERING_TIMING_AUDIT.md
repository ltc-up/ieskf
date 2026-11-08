# Production timing and research timing audit
Production source was read only at the current checkout; content identities are in `source_audit.json`. The pre-existing production NDT YAML edit was not changed. Configuration files describe checked-in defaults, not proven historical runtime parameters.

| Behavior | Production source / function | Research implementation | Qualification |
|---|---|---|---|
| State | `ekf_localizer/src/ekf_module.cpp`, initialization / state transition | `geometry_ieskf/state.py`, inertial.py | Production planar x,y,yaw,yaw_bias,vx,wz + separate Z/roll/pitch filters; not a 15D inertial model. |
| Prediction delay | `kalman_filter/src/time_delay_kalman_filter.cpp::predictWithDelay` | `geometry_ieskf/replay.py::EventReplay.run` | Production shifts augmented states and preserves cross-time covariance; research sorts all Header events and propagates ZOH. No augmented lag covariance. |
| Delayed update | `updateWithDelay` selects measurement H block at lag; joint K changes current and past states | No delayed local correction/repropagation scheduler | NOT_IMPLEMENTED in online EventReplay/ROS; separate offline prototype below. Cannot port a six-state A/H into 15D. |
| Delay indexing | `ekf_module.cpp::accumulate_delay_time`, `find_closest_delay_time_index` | Exact timestamps / pose interpolation | Production nearest accumulated actual-dt slot, out-of-range rejection; research map interpolation has 100 ms max bracket gap, 2 s bounded history and 500 ms observation age. Different semantics. |
| Queues / smoothing | `aged_object_queue.hpp::pop_increment_age`; timer in `ekf_localizer.cpp` | Research candidates consumed once by correlation group | Production pose repeated five times, twist once by checked-in defaults; `measurement.cpp` scales R by smoothing count. No implication of five independent measurements. Research does not reproduce this smoothing. |
| Timer clock | `ekf_localizer.cpp::updatePredictFrequency` / timer callback | Offline Header timeline | Production steady-clock dt, current ROS timestamp for delay and publication, warnings/cap for large dt. Offline replay has no actual executor jitter. |
| Z delay | `ekf_module.cpp::compensatePoseWithZDelay` | Full 15D propagation, no map update | Production pitch/vx-based Z compensation is not inertial delayed fusion. |
| NDT feedback | NDT callback queues map pose when enabled | AL shadow accepts copied pose history only | Historical fused output is not independent local odom. |
| Global delay | Not the same mathematical object as local lag filter | `ndt_global/shadow.py::PoseHistory.at`, `ShadowCorrector.consume` | Computes M(tau) inverse(O(tau)); never pairs delayed M with current O. Does not repair delayed local measurements. |
| ROS2 | Timer/callback queues and mutexes | `ros_shadow.py`, `ros_global.py` isolated String research adapter | Saved-event processing was demonstrated previously. Actual callback order, online lag correction, backlog and causal Deskew/NDT loop equivalence remain NOT_VALIDATED. |

Checked-in production defaults: 100 Hz prediction, 15 extended steps (nominal ~0.14 s oldest lag, actual dt governs), pose smoothing 5, twist smoothing 1, additional delays zero. Runtime overrides must be acquired before an equivalent production replay can be claimed.

## Same real input: three groups
Executed `timing_comparison.py` on 11,680 recorded fused pose samples and 2,292 recorded NDT pose samples from the R2 fixture. This is a temporal pairing experiment; those poses are contaminated by production global feedback and are not independent odom or truth.

1. Header-order oracle: interpolate the recorded pose at each NDT Header.
2. Recorded-arrival order: pair the NDT with the latest available recorded pose. This intentionally demonstrates the incorrect latest-state subtraction.
3. Recorded-arrival order plus historical compensation: maintain bounded history, interpolate at NDT Header, check 500 ms age and bracket contract.

All 2,292 pairings have brackets. Group 2 differs from group 1 in correction translation: P50 2.968 m, P95 18.704 m, P99 30.773 m, max 38.808 m. Group 3 agrees with group 1 to numerical zero. These are **correction differences, not vehicle localization errors**; large coordinate lever effects can amplify a pose/time mismatch. NDT receipt-minus-Header P50 43.106 ms, max 102.543 ms.

Recorder receipt order is only a proxy for actual node callback arrival. Callback start times, production queue ages, thread scheduling and runtime overrides are not recoverable from these exports. Consequently the requested actual-callback/online equivalence test is NOT_VALIDATED, and online/real-time closed-loop qualification fails. A runnable historical map-pairing comparison has been delivered; a local 15D fixed-lag checkpoint/repropagation scheduler and its full-input comparison remain separate required work. No claim that Map/Odom history implements `updateWithDelay` is made.

## Independent local-delay implementation and real comparison
`ndt_global/delayed_local.py::DelayedLocalReplay` now provides a **separate offline prototype**: a 500 ms bounded checkpoint history (hard 10,000-event cap), complete state/P/held-IMU checkpoints, ordered suffix repropagation and duplicate identities. It reuses existing inertial and scalar-wheel equations without Q/R changes. An observation is reapplied once from its pre-observation checkpoint, not added a second time to the posterior containing it. Unavailable IMU input defers the suffix; no unpropagated state is published. Inputs beyond the forward-history budget are rejected. This is checkpoint replay, not production lag-augmented covariance or smoothing.

Executed all 43,441 real Bosch/wheel events over the same 144.85 s usable original-loop7 prefix from a common first local output. Naive recorded-arrival processing rejects 2,263 late events and one IMU gap. Its mixed-unit 15D difference norm is diagnostic only, not a position error or accuracy metric. The corrected delayed runner handles 2,265 suffix repropagations and one deferred input; final nominal state and all 225 covariance entries match the Header-order reference exactly (max P difference 0, no pending inputs). Maximum buffer 153 events. See `local_timing_results.json` for actual performance.

The first implementation rejected the temporarily unpropagatable wheel event; its final state/P did not match. This **failed execution is preserved** in `local_timing_attempt1_failure.json`, with a regression test for late-IMU unblocking. Waiting within the declared history budget resolved the scheduling defect; it does not fix inertial drift, Z observability or sensor calibration. No 15D state row/cross-covariance was deleted.

This demonstrates same-input offline repropagation equivalence over the tested prefix only. It does not validate callback receipt reconstruction, delayed online startup, tail behavior beyond missing IMU, a whole lap, production six-state smoothing equivalence, ROS2 deadlines or a causal Deskew/map closed loop. Production replacement remains NO-GO.
