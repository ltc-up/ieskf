# Final AR conclusion — functional GO, normal research availability GO
1. AQ2 original40 restored:40processed,30 XYZ/yaw updates; independent F40 repeat native seeds/finalposes/full15D deltas exactly equal60s prefix.
2.2.70s interruption reproduced exactly:35iteration budget ends before unchanged strict stability criteria; repeated rejection removes observations.
3. Rotation second derivative sign error verified and enters optimization Hessian, but corrected-only behavior unchanged in tested frames. Not the observed main cause; production unchanged.
4. Interaction demonstrated: optimizer needs more iterations with own prediction, then XYZ/yaw allows pitch/bias feedback drift and bad matching. Frozen100budget+existing full map SO3 observation resolves tested continuation. Not a claim all IMU calibration problems are solved.
5.20s401/401frames and updates.
6.60s1201/1201frames and updates.
7. Prior loop44/map0105 lap1468/1468frames and updates, native endpoint span73.350s within frozen73.4345s lap window. Genuine raw CUDA Deskew/NDT/own15D feedback, not frozen pose replay.
8. Lap NDT/ESKF acceptance100%; max adjacent accepted gap51.004ms, initial anchoring154.063ms; no later lost interval. Correctness rate unknown without independent labels.
9. No lap outage/recovery triggered: recovery success N/A, not proven robust recovery.
10. Zspan10.324m,vz−2.098..2.190m/s, finite bounded attitude/bias and no giant wheel Z correction. True height accuracy NOT_VALIDATED.
11. Uses trusted legacy loop44 recorded converted wheel scalar; old raw-CAN producer build not inferred from modern eav25 topic names. No loop8/GNSS fusion.
12. Full pose changes full15D via complete P, affects following Source/seed; mathematical and actual-event evidence retained.
13. Same-time production XY median1.246m/P952.505m,max2.839m; yaw differences up to~2.75deg. Not independent accuracy improvement.
14. Matcher P95~5.11ms promising; research section max359.88ms, no ROS20Hz qualification.
15. Remaining blockers: statistically unqualified engineering R/reference and sensor initialization/noise contracts, untested recovery/wrong peaks outside this normal lap, persistent-adapter portability, full real-time scheduling and output ownership validation.

Functional GO and availability GO apply only to the declared research normal-window criteria, full-pose candidate and map0105. Independent precision NOT_VALIDATED;20Hz real-time NOT_VALIDATED; production deployment/replacement NO-GO. All earlier failures retained. No production modifications or TF publication.
