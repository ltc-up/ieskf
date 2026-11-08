# A–E comparison qualification
All raw extraction and local runs use the preselected whole inputs and the unchanged Phase V configuration inherited by AL. There is no controlled full-lap accuracy A/B result.

| Group | Actually executed / scope | Qualification |
|---|---|---|
| A production NDT+EKF | Recorded pose and NDT topic extraction; receipt and Header statistics | HISTORICAL_REFERENCE_ONLY. Runtime NDT enable, active map SHA, seed, accepted/fallback publications and queue ages not established. Production fused pose is used once to initialize B, never as ongoing odom or truth. |
| B local 15D | Real Bosch + existing scalar wheel model using recorded static TF, whole-record attempt for qualified timestamps | Actual outputs and full P saved privately; failures retained in per-input results. Model/noise/gravity/bias initialization remain uncalibrated. Kistler not independently double-fused. |
| C local15D + NDT Map/Odom | AN manager executed over real recorded NDT poses and B pose history | NOT_QUALIFIED as new NDT localization. Public pose topic lacks convergence/source/seed/map/diagnostic qualification, so UNKNOWN is preserved, no trusted anchor. No fresh full-lap CUDA/Deskew run is claimed. |
| D local15D + GNSS | Real raw observations submitted as unqualified to manager | NOT_QUALIFIED: raw LLA is not a map-base position, fix/model/clock/datum/lever/noise unresolved. |
| E multisource | Same manager schedule receives C and D, all rejection reasons counted | Executable qualification/state path, not qualified statistical fusion or valid normal-lap global localization. |

Global scheduling uses actual recorder times for map/GNSS but idealized Header availability for B outputs. This intentionally does not claim actual live callback ordering. No other branch's future state is used for a new match seed: **no new full-lap matches were generated**, because the frontend/map/reference prerequisites were not established. Existing AM/AJ fits are separately historical evidence, not E outputs.

Rejecting everything preserves semantic integrity but provides 0% trusted localization coverage; it is a functional failure for localization, not successful robustness. No fabricated R converts unavailable observations into an apparent success.
