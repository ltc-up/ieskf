# Preserved failures and incomplete qualifications
1. Primary loop35: all 75,754 scalar-wheel Header timestamps are zero. No recorder-time substitution or wheel fusion. Run returns NOT_QUALIFIED.
2. Original loop7: attempted entire 1433 s input; existing IMU-gap gate stops at 144.86 s. Raw maximum IMU gap 3.227 s. No selected restart after the gap. Local Z span before stop 640.226 m, final vz -8.953 m/s. These are uncalibrated estimator outputs, not truth error.
3. Whole known-fault loop44: 917.07 s processed, 91,707 outputs; final wheel/IMU tail mismatch triggers IMU-gap stop. Z span 70,720.196 m; max wheel dZ 150.939 m; model sigmaZ 770.446 m. Isolation does not make this acceptable local odometry.
4. First checkpoint-replay attempt dropped a wheel input when its IMU was late: nonzero final state/P discrepancy. Full aggregate failure retained. AO corrected it with a pending suffix and added regression coverage; no noise tuning.
5. Historical NDT public pose lacks full candidate qualification. No newly regenerated, causal, full-lap CUDA/Deskew/15D/global chain was established. Map/GNSS/reference qualification is not inferred from a filename or low score.
6. Actual node callback order, historical runtime parameters, complete production queue behavior, ROS2 end-to-end latency, independent accuracy and vehicle acceptance limits are unavailable. No online equivalence or real-time PASS.
7. AP cannot have full robustness acceptance because AO has no qualified normal baseline. Zero trusted anchors is a localization availability failure, even though no false trusted output is emitted.

Per-input JSON and execution records include unsuccessful attempts; no failed frames or intervals were removed. Additional backup results are included in FULL_LAP_EXECUTION and its JSON, including any further failure. Source/library/method preservation does not imply an entire requested deployment was achieved.

8. Original loop8 backup: 2455.68 s / 245,568 outputs, terminal IMU-tail gap before remaining wheel input. Z span 310,906.212 m and max wheel dZ 597.146 m. No full local-state quality qualification despite numerical propagation and PSD. Its entire attempted input is accounted for.
