# Executed validation

-195 existing Python tests PASS after migration, using copied original frozen configuration and new-volume data bindings.
-6 new diagnostic tests PASS: long monotonic sequences are not oscillation; sustained recurrence is required; nonfinite/short traces are not proof; XY-yaw cross coupling and Hessian sign are retained.
-84 independent KD radius queries PASS against brute force: exact boundary, negative coordinates, planar data, invalid/empty voxels.
-40 real CUDA one-step support/score/gradient/Hessian checks exactly match the existing radius oracle; repeated after public-builder refactor. DIRECT7 support differs, as expected.
-25 real CUDA migration checks exactly reproduce stored P0/KD poses, gradients, Hessians, objective, exits and iterations using newly compiled new-volume binaries and new-volume source/target paths.
-1468-frame original15D arrival-order counterfactual baseline prior means/R/P maximum difference0. Existing history/dedup tests are included in195 above.
-Production AgedObjectQueue real-header C++ test PASS: five attempts, old queue survives rejection of new input, failed update consumes age. No production code modified.
-Selected MT serial max100 vs max35 run:1468 native seeds/final poses/Hessians exactly equal; this is a within-candidate cap/parity check, not original AY historical whole-lap equality.
-Original AY historical ledger comparison: optical inputs exact SHA match, but native results are not byte-exact. Detailed differences retained; no exact historical full-lap reproduction claim.
-Migration: full private/input checksum rsync dry-runs completed with empty differences; Git mirror fsck and bundle verify PASS.

Source/build logs, full failure prefixes, runtime records and migration evidence stay under the actual new-volume directories listed in STORAGE_LAYOUT.md. All per-frame deadline overruns are retained. Passing numeric regression does not establish peak correctness, statistical calibration, online ROS executor timing or control safety.
