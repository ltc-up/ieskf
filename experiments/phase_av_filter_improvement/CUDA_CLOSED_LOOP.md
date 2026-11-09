# Real own-state CUDA feedback

Fresh direct legacy40 and wrapper legacy40 reproduce AR exactly: seed, prior fullP, final CUDA pose, K and15D delta all max difference0. This precedes changed-algorithm evaluation; no frozen pose sequence was injected as a measurement.

Every candidate frame uses original three-LiDAR raw clouds, authorized persistent Deskew, its own historical motion/lever, its own seed/ROI, original map0105 Target and original fixed100-iteration CUDA binary. New observed pose commits at original Header time and repropagates later IMU/wheel. Original Q/R wheel/config/vehicle, matcher derivatives and per-point optimizer are unchanged. GNSS never enters the loop. Initialization is the same original lawful historical initialization; afterward no oldEKF state or future branch result enters feedback.

|Branch|Actual frames /1468|Accepted|Outcome|
|---|---:|---:|---|
|B0 packet-only|1468|1468|Mixed accuracy; full repair failed|
|V1 all-or-nothing ambiguity + hardNIS|881|796|2s loss gate; failed|
|V2 consensus projection + hardNIS|1159|1112|2s loss gate; failed|
|V3 robust consensus|1468|1468|Single-lap function complete, full repair failed|

V3 versus legacy:1467/1468 Source SHA change; first Source remains equal. Final NDT translation difference median7.97mm, P950.07202m, max0.48788m. Changed Deskew/initial seeds therefore caused genuine rematching, not fixed-observation replay. Small median differences still matter for some basins; neither convergence nor closeness certifies the correct map peak.

Three hypotheses from one cloud share one correlation group and are fused as one projected central observation, never three independent measurements. On ambiguous frames W removes all tested candidate-difference combinations; partial constraint status is explicit. It does not prove the true pose is inside that candidate set.

V3 original500ms multilap run completed1480 accepted frames before history assertion. Explicit2.1s retry completed3458 accepted frames then failed again as processing backlog grew. First1468 native Source/seed/P/match/delta/postresetP are exactly identical across cache settings. All failed Source/Target identity, hypotheses, state/P, per-transaction JSONL and error logs remain in persistent ignored storage. The retry is not a completed4134-frame experiment.
