# AU qualification limits (retained across stages)

AU1 found that AT lacks complete pre/post IMU and wheel mean histories. It could not support exact propagation attribution by itself. A separately instrumented fixed-observation replay now reproduces all1468 saved native priors and published complete states/P exactly. No failed AR run was overwritten.

Original pre-NDT-ingest CPU sub-boundary is absent. Reconstructed available/complete envelope passes state/P equivalence but is not proof of a real ROS arrival/executor history. Repropagation trace rows are execution revisions; only the final canonical sequence may be accumulated as information.

Physical GPS latency, antenna/INS/map reference and true Bias/attitude calibration remain unqualified. NDT is not ground truth. No sensor-only physical causal attribution or Q/R change follows from state consistency. Further stage outcomes are appended without deleting these limits.

AU2 initial analysis was stopped because lazy NPZ field access repeatedly decompressed entire arrays inside the diagnostic loop. Arrays are now loaded once; cached rerun completed. This was a new analysis-script I/O defect, not a frozen estimator failure; first/corrected private logs are retained. No partial numbers were reported as results.

Independent full-state matrix checks and nominal/balance closures pass. The .1m-scale current innovations are discrepancies to NDT, not known true errors. Axis transport is mandatory; omission would falsely attribute part of curved-frame coordinate change to new propagation bias.
