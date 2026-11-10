# BB pre-execution scope

Frozen parent BA3606769; AY raw opticalXY is the estimator baseline. Global CUDA-NDT100/map0105/crop/poseR and physical15D remain unchanged. Scan-to-scan relative SE3 is used only to produce interval velocity and quality; no relative pose, pseudoposition, map optimization, sliding window or additional independent pose update.

First audit FAR derivative and replay frozen actual Deskew clouds for timing/frame/noise diagnosis at50/100ms. This is fixed-input diagnostics only. Then use own-state real Deskew clouds for every actual closed-loop trial, including1468 and4134 without reset. GNSS evaluation only, fixed0/75/150ms, all fixed segments0–9.

Unknown cross-correlation precludes claiming an independent calibrated velocityR. Primary conservative usage is velocity integrity evidence/optical veto, not extra Kalman information. Freeze its rules before reference evaluation. Relative interval velocity is not an instantaneous endpoint sample. If evidence cannot support a state measurement, do not force an extra fusion. Preserve no-effect outcomes and adverse results. Quantify overhead and500ms-history failures honestly. No future cloud, main/production edits, stash operations or new datasets.
