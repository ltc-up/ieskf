# Failures and blocked qualifications
1. AO had no qualified normal-lap baseline. AP full robustness acceptance is forbidden, regardless of unit/injection outcomes.
2. Global control has no qualified anchor or trusted publication. All nine fault cases retain0% coverage; no real GNSS/NDT recovery or quantified accuracy improvement can be evaluated.
3. First wrong-NDT shadow injection returns NO_BRACKET at the frozen startup frame. The probe was not relocated. Second run adds rejection-reason instrumentation; first run is retained. Synthetic regression is labeled separately.
4. GNSS raw-LLA outlier cannot reach a numerical innovation gate because its map/reference/fix/time contract fails first. No claim of detecting a large real GNSS error.
5.20m Target deletion lowers first-frame nearby support3.2percentage points, but all ranks remain6 and geometry classifications remain VALID_CANDIDATE. Automatic partial-hole detection has not passed. Real map-hole extent/independent labels remain missing.
6. Historical CUDA endpoints have no independent true/wrong labels. Two multi-peak groups are not a validated real wrong-peak detector. No new causal full-lap matching/GPU performance is available.
7. Partial-LiDAR combinations and dropouts require original per-source Deskew/Source recomputation; NOT_VALIDATED for all requested subsets. Removing NDT observations does not simulate a missing raw LiDAR.
8. Long local15D vertical failures persist unchanged; no Z/vz clamp or P surgery. Global isolation preserves the failure as well as the local state. Inherited local_continuous flag is not physical state-health validation.
9. Actual online callback timing, end-to-end20Hz/100Hz, independent precision and production replacement remain NO-GO / NOT_VALIDATED.

Automated tests pass134/134; failed qualification and ineffective probes are retained independently of that software test result. No failed case was hidden or converted to GO.
