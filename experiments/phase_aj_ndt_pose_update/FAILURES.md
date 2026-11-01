# Failures retained

- Main experimental failure: only66/284 C updates submit. There are129 NDT budget exits,87 unsafe proposals,2 invalid historical-R fits; longest rejection run108 frames. No parameter change rescues them.
- No direct height measurement: C Z range10.047m, final sigma Z47.596m, and actual unchanged-wheel corrections up to4.137m Z/.560m/s vz. A/B/C state stability is not reducible to LiDAR delta alone.
- 3 callbacks exceed50ms. Offline throughput is not a real-time qualification.
- First summary script assumed Diagnostics retained raw timing samples and raised KeyError. The failed log and generated matrix audit were archived; only the reader was fixed. The same153 saved-prior cases were audited twice across those two analysis passes; this is306 frozen matrix calls, not another recursive trajectory. No sensor replay/NDT was repeated.
- A read-only causal audit initially required bitwise double equality after serialized quaternion reconstruction. Roundtrip error is2.84e-14; the check now uses a numerical serialization bound, while all284 actual backend float32 seeds still match exactly. No seeds or states changed.
- Final source-policy audit identified an omitted existing publisher floor for a never-exercised small-fitness edge. Its original1e-5 value was restored; minimum observed fitness .088709 and all153 recorded R matrices remain exactly equal. Original executed code and initial ten-test log remain local; final11 tests pass. This is not R tuning or a second continuous run.
- Isolated inner ESKF timing was not retained in the original callback; posthoc timings are labeled. Generic inherited GPU metadata was misleading for an external service and is corrected in the public summary, not treated as evidence of CPU-only execution.
- Early read-only searches guessed nonexistent files before inspecting actual source paths. No failed search result was used as code evidence.

No new noise/velocity model, stronger NIS gate, production modification, map regrouping, geometric optimizer iteration or conditional initial-state reset was introduced. All original failures and actual proposals/submissions stay in ignored evidence. No independent truth, statistical calibration or deployment qualification exists.
