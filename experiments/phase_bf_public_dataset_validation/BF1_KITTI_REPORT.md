# Phase BF1 — KITTI0002 completed, functional qualification only

Baseline143ce47adfb60cbc8d1f069d4945b33641e1ca96. Only the three approved archives were acquired, total814833599bytes. All ZIP entries passedCRC, all expected lengths match, SHA256 identities retained inBF1_MANIFEST. No other scene/RACECAR download, historical cleanup, production change or optimizer/Q/Bias tuning.

## Actual input and fixed map

77 finiteXYZI scans,846 finite30-field OXTS records. RawIMU interval median9.997412ms, min9.602191ms/max10.353380ms; no25msgap. LiDAR median103.176933ms, scan duration median103.176340ms. SyncOXTS epoch is11.747884–27.618032ms AFTER same-indexLiDAR, median17.524373ms: index-pairing is invalid. Initial preparation using that pairing is retained but not matched. Canonical preparation interpolates rawOXTS navigation poses at actual scan epochs for mapping/evaluation only; no frame split change.

Official body axes/units are used: ax/ay/az m/s² and wx/wy/wz rad/s. Mean body force[0.416705,0.158869,9.795146]m/s² is consistent with gravity-containing specific force; offline comparison to navigation-velocity derivatives has axisRMSE[0.078883,0.164887,0.150825]m/s². This is a consistency test, not proof that exportedIMU is independent of the receiver's GNSS/INS processing. No rawADC or wheel source is invented.

The official IMU→Velodyne rigid calibration is inverted once; source points are expressed inIMU axes/origin, NDT returns map-from-IMU. Existing ExternalPose uses zero lever in this declared observation chart, not a guessed rear-axle transform. No private racecarTF reused. The initial pose and velocity are taken once from mapping-frame0; all later motion uses inertial channels and NDT. No test navigation value enters seeds, weights or updates.

Map scans0–19 only,0.2m map downsampling fixed before matching; NDT resolution1.5m unchanged. Target171558points from2419753mapping points. Tests20–76 all retained. Offline reference-poseNN overlap at1m is≥55.47% for all57scans (frame20≈99.98%, frame76≈55.47%). This coarse ground/surface overlap does not certify longitudinal observability. No test scan is added to map. Independent map/GT is absent: **functional reference-assisted validation**, not independent accuracy certification.

No explicit point firing timestamp exists inXYZI. Start/end/forward-epoch scan timestamps exist, but exact firing order/motion compensation provenance is unqualified. No fabricated per-pointtime or additionalDeskew is used. Consequently this does not certify the production fullCUDA-Deskew pipeline.

## Execution and fairness

B0/B1_run1 each processed57scans using unchangedDIRECT7/GLL3, step0.2, eps0.01, relativeobjective0.001,35iterations, q50 and original15D Q/Bias/domain checks. No GMF/GICP/newregistration. Both priors contain full15D state/P. Before any live submission each frame records B0/B1 fullR/K/P and15D candidate deltas on the SAME prior and SAME finalmatch; these are Shadow, not carried states.

The two first causal runs used measured wall-time arrival simulation, so timingjitter can confound covariance attribution. A declared control repeats onlyB1 with B0's recorded57arrival epochs. All epochs are verified equal; primary table below is B0_run1 versusB1_fixed_arrival. It is a deterministic causal delay replay, not a liveROS2deadline test. B1 measured compute would miss that imposed schedule in31/57cases; that is retained, not hidden. OriginalB1_run1 remains separately evaluated. Actualcompute still measured independently.

Observation timestamps remain scan epochs; availability includes spin completion and measured processing. Existing500ms ManagedReplay inserts past observations and repropagates original heldIMU inputs. UniqueIDs and terminal rejections retained. No repeated smoothing of the same posterior, no pose overwrite, no resets between segments. Both complete57 and accept46; frames66–76 all MAX_ITERATIONS and rejected. Prediction-only tail since last validmapupdate≈1.134575s.

## Reference comparison (0ms, all57published states)

| Metric | B0 Legacy | B1 Native Raw, common arrival |
|---|---:|---:|
| XY RMSE (m) | 0.220625 | 0.196802 |
| 3D ATE RMSE, common initial frame (m) | 0.272147 | 0.250446 |
| Longitudinal RMSE (m) | 0.189265 | 0.156756 |
| Yaw RMSE (deg) | 0.074206 | 0.087840 |
| Z reference RMSE (m) | 0.159339 | 0.154894 |
| Vz reference RMSE (m/s) | 0.043396 | 0.038282 |
| 1s RPE translation RMSE (m) | 0.152669 | 0.146221 |
| Lateral absolute P95 (m) | 0.163078 | 0.173032 |

ATE is common-initial-map-frame discrepancy, no trajectory/scale fitting. RPE uses47approximately1spairs with60mstolerance, not a fictitious100mbenchmark. Positive sensitivity offset means compare state at t with OXTS reference at t+offset; no fitted timing shift. XYRMSE0/75/150ms: B0=.220625/.970981/1.794849m; B1=0.196802/0.937795/1.761988m. These shifts are sensitivity perturbations, not a calibratedGNSS delay forKITTI.

RawNDT proposals (including11invalid) haveXYRMSE≈2.280m/2.205m; converged46-only≈.1758/.1761m. Both denominators are reported; removing failed11would misrepresent full matching capability. Tail longitudinal drift/iteration exhaustion is compatible with reduced useful map overlap, but a wrong local optimum is not established solely by MAX_ITERATIONS. No accepted obvious>1mwrongpeak example is certified here; general wrong-peak safety remainsUNKNOWN.

Native lowers this shortsceneXY/Z-reference discrepancies slightly but worsens lateralP95 andYaw. It does not qualify replacingLegacy. Reference-assisted map and sharedOXTSinertial/reference errors preclude independent statisticalcalibration/NEES or generalization claims. See covariance/performance reports for complete qualifications.
