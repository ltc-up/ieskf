# Preserved AQ2 failures
1. Missing legacy raw paths: first run aborts before a match. Raw originals subsequently recovered by bag ID/CDR SHA; no old motion pose reused.
2. Per-frame process/CUDA cold startup: first match is valid/converged but arrives outside500ms history; subsequent queued Source lacks brackets and asserts. Failed transaction/log retained. Persistent private service/map initialization fixes lifecycle, not filter noise/gates.
3. New raw extraction first invocation overwrote sourced ROS PYTHONPATH and failed import. Corrected invocation appends src to ROS path; no algorithm/input change.
4. B/D short retention exceeds500ms continuation criterion; only C advances.
5. C20 processes55/401 frames, accepts32 then stops at2.70s after0.55s without accepted NDT. MAX_ITERATIONS candidates can have low fitness; neither correctness nor convergence is fabricated. No full-lap/fault run follows failed normal continuation.
6. First C20 summary mistakenly used requested20s to label processed duration/no-observation interval. Original summary retained; corrected summary explicitly separates55 evaluated frames/2.70s from401 requested/20s. No backend result is changed.
7. Engineering R, hardware/reference/bias calibration, independent accuracy and full real-time qualification remain incomplete. Numerical recursion/geometry rank cannot turn these into GO.
