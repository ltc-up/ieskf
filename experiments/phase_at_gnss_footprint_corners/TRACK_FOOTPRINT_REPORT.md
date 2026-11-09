# AT2 — Actual boundary computations and limits

**CONDITIONAL model result; physical tyre/safety qualification NOT_VALIDATED.** Frozen AR1468 frames and1197 valid raw-rebuilt GNSS frames were processed, without rerunning or modifying localization.

AS source is retained: static_info.launch.py→boundary.launch.py→boundary_publisher.cpp, yas_north left/right CSV, and AS fixed BaseLine segments. LINE_STRIP vertices are used as full line segments, including long sparse segments. Both rings are simple, nonintersecting and nested (corner_quality.json). There is no trajectory alignment fit. Historical A2RL_MAP selection, boundary survey uncertainty and map0105 physical drivable-edge certification are missing. Planning raceline/corridor is not substituted for physical boundary.

The generic footprint algorithm checks full polygon-boundary intersections and containment, including a polygon enclosing the track hole. Touching is flagged; contained shapes have positive boundary separation. Intersecting shapes report zero boundary separation and an invalid-containment flag, not a fabricated penetration depth. All shapes were contained in this run.

| Trajectory | Tested visual boxes | Not contained / boundary intersections | Minimum model boundary clearance m |
|---|---:|---:|---:|
| New15D |1468|0 /0|.440|
| ProductionEKF |1468|0 /0|.139|
| GNSS engineering |1197|0 /0|.619|

These are **historical visual-box assumption** results, not measured body/tyre results. Rear-axle and front-axle point distances are separately retained in statistics.json. Minima frame indices/corner IDs and full private polygon records permit locating each event without publishing trajectories. No suspected noncontainment events occurred under this box assumption; continuous-event duration is consequently zero at sampled frames. Continuous interframe swept containment was NOT_EXECUTED.

Private corner3/9 overlays draw both estimators' own transformed box polygons against actual CSV segments. Missing tyre footprints are explicitly labelled unavailable; no four-wheel rendering implies tyre certification. GNSS uses its interpolated INS orientation and valid reference intervals; that orientation is not independent surveyed yaw. Position/time/lever, tyre/body dimensions and boundary survey uncertainties lack certified error bounds. Even the positive .139 m modeled clearance cannot establish a real safety margin. Actual tyre line crossing and safety GO remain NOT_VALIDATED.

The final box projection includes all eight 3D vertices and their convex map-XY silhouette, including visual-origin height and each pose's Roll/Pitch/Yaw. Minima occur at new15D frame792/section8 and production/GNSS frame1271/section2; these are not the fixed difficult right sections3/9.
