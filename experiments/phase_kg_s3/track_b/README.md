# B shadow implementation

This directory is original research adapter code. It does not contain production CUDA source, real point clouds, extrinsics or trajectories. `local_match_service.cpp` links a separately authorized private CUDA matcher; its include headers and library must originate from the same build to avoid ABI mismatch.

The common causal replay runner calls `bridge.ShadowGroup.on_frame(...)` synchronously while current Deskew dense scratch is still valid. Run one variant per complete replay for isolated timing. No B result is fed into the global state, U or another B variant. Initial O retains U roll/pitch; only initial yaw and translation are removed by fixed O_T_U.

`B0`: preceding actual scan, including failed previous registration. `B-M`: up to three preceding scans within one second, accumulated using that branch's O anchors. `B-K-MR`: separate authorized coarse3→fine1.5 NDT comparator sharing B-K history, with fine failure falling back to prediction, never coarse pose. `B-K`: bounded overlap insertion, low-overlap removal and latest contribution versus redundancy score; this is a GLIM-inspired selection comparator, not GLIM native odometry. `B-P`: independent U motion only, without target preparation.

All current Sources are immutable copies of the same common A0 source values. Dense historical input is Deskewed base cloud BEFORE Source rectangle, downsampling and 5000 cap. Historic clouds are transformed into gravity aligned O, cropped at current O prediction by radius 120 m and relative z [-10,30] m, then voxel averaged at .8 m. Target and Source have distinct rules.

Each local service request constructs, builds and destroys its own local GPU matcher. The global matcher and immutable global map stay untouched. Constructor, build, quality evaluation, serialization, destruction and CPU Target preparation must all be included in full-cycle timing.

`validator_v1.json` is a frozen uncalibrated engineering domain rule, not a reliable-correct-peak classifier. Distinct neighbor source count is used, never sum of seven neighbor pairs. Innovation compares candidate to current local O prediction, with actual scan dt. Every rejected candidate is preserved. Acceptance only updates O pose, not U velocity, bias or covariance. High corrections may be falsely rejected. Source Deskew retains common global-map history correlation.

Run CPU checks:

```
python3 test_local_odometry.py
python3 test_bridge.py
```

`glim_diagnostics.py` implements signed source-normal/view-ray surface filtering and two-resolution occupancy diagnostics separately. Surface diagnostics do not alter NDT score or filter the current Source. B-K-MR separately implements true two-level NDT registration. Native normal sign convention differs and is explicitly documented; native VGICP joint optimization is not claimed.

`run_stream.py` consumes a common runner NDJSON stream for a single variant. It requires complete-prefix identity and causal U declaration. These declarations are audited by the common runner; this adapter alone cannot establish causality or create missing history.
